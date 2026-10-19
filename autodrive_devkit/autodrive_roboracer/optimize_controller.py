#!/usr/bin/env python3
"""Automated simulator-in-the-loop parameter search for wall_follow_node.

Runs on the HOST (Windows/PowerShell). For every trial it pipes a small worker
script into the running ``penn-challenge`` container with ``docker exec``; the
worker (ROS 2 Humble, rclpy) checks telemetry, resets the simulator, starts a
fresh ``wall_follow_node`` with ``-p`` overrides, measures laps, stops the node,
and prints one JSON result. The controller, the bridge, and the existing tests
are never modified; parameters are startup-only, so the node restarts per trial.

Stages: A baseline repeats -> B conservative joint search (max_throttle held
near the known-stable range) -> C higher max_throttle, ceiling raised only
after a valid run -> D local refinement + repeat runs of the best candidates
and of the baseline. Uses Optuna TPE when importable, else seeded random search.

Results (JSONL = source of truth, CSV, summary.json, failure diagnostics) are
written after every trial under data/optimization/<run>/ and are git-ignored.

Examples (PowerShell, from the repository root):
  python autodrive_devkit/tools/optimize_controller.py --dry-run
  python autodrive_devkit/tools/optimize_controller.py --check-telemetry
  python autodrive_devkit/tools/optimize_controller.py --baseline-only
  python autodrive_devkit/tools/optimize_controller.py --max-trials 8
  python autodrive_devkit/tools/optimize_controller.py --max-trials 20 --resume
"""

import argparse
import atexit
import base64
import csv
import hashlib
import importlib.util
import io
import json
import math
import os
import platform
import random
import shutil
import signal
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import fields, replace
from datetime import datetime
from pathlib import Path

try:  # byte-range lock on Windows, flock elsewhere
    import msvcrt
    fcntl = None
except ImportError:
    msvcrt = None
    import fcntl

sys.dont_write_bytecode = True

DEVKIT = Path(__file__).resolve().parents[1]
CONTROLLER_PATH = DEVKIT / 'autodrive_roboracer' / 'wall_follow_node.py'
DEFAULT_RESULTS = DEVKIT / 'data' / 'optimization'
CONTAINER_NODE_PATH = ('/home/autodrive_devkit/install/autodrive_roboracer/lib/'
                       'python3.10/site-packages/autodrive_roboracer/wall_follow_node.py')
WORKER_PATH = '/tmp/of_trial_worker.py'

# Verified configuration (user-reported). Passed explicitly on every trial so
# results do not depend on whichever defaults the installed node happens to use.
BASELINE = dict(
    optimization_enabled=True, adaptive_corner_throttle=0.14, max_throttle=0.26,
    min_throttle=0.12, straight_steering_threshold=0.0, kp=1.2, kd=0.08,
    desired_wall_distance=0.65, lookahead_distance=0.8,
    dynamic_lookahead_enabled=False, steering_smoothing_tau=0.0)

# Parameters the controller actually exposes and that we are willing to search.
# NEVER searched: front_obstacle_safety_distance (emergency stop), scan_timeout,
# min_valid_fraction, wall_side, fallback flag, ki, dynamic lookahead/smoothing.
SPACE = {
    'max_throttle': (0.24, 0.40),
    'adaptive_corner_throttle': (0.10, 0.20),
    'corner_slowdown_threshold': (0.25, 0.60),
    'preview_slowdown_distance': (1.5, 3.5),
    'front_slowdown_distance': (1.0, 2.4),
    'preview_path_half_width': (0.20, 0.40),
    'preview_half_angle_deg': (30.0, 50.0),
    'lookahead_distance': (0.6, 1.2),
    'throttle_rise_rate': (0.3, 2.0),
    'kp': (0.9, 1.6),
    'kd': (0.04, 0.14),
    'desired_wall_distance': (0.55, 0.80),
}
STAGE_B = {  # conservative: speed-selection parameters, max throttle near stable
    'max_throttle': (0.25, 0.28),
    'adaptive_corner_throttle': (0.12, 0.18),
    'corner_slowdown_threshold': (0.25, 0.50),
    'preview_slowdown_distance': (1.5, 3.5),
    'front_slowdown_distance': (1.2, 2.2),
    'preview_path_half_width': (0.20, 0.40),
    'throttle_rise_rate': (0.3, 1.5),
    'lookahead_distance': (0.7, 1.0),
}
STAGE_C = {  # adds steering gains; max_throttle ceiling is dynamic
    'max_throttle': (0.26, 0.40),
    'adaptive_corner_throttle': (0.10, 0.20),
    'corner_slowdown_threshold': (0.25, 0.60),
    'preview_slowdown_distance': (1.5, 3.5),
    'front_slowdown_distance': (1.0, 2.4),
    'preview_path_half_width': (0.20, 0.40),
    'throttle_rise_rate': (0.3, 2.0),
    'lookahead_distance': (0.6, 1.2),
    'kp': (0.9, 1.6),
    'kd': (0.04, 0.14),
}
STAGE_SPACES = {'B': STAGE_B, 'C': STAGE_C, 'D': STAGE_C}
FAIL_BASE = 15.0   # objective for an invalid trial = FAIL_BASE + missing laps
SIG_DIGITS = 4


# --------------------------------------------------------------------------
# Worker executed INSIDE the container (python3.10, rclpy). Kept in this file
# so the optimizer is a single script; it is piped to /tmp/of_trial_worker.py.
# --------------------------------------------------------------------------
WORKER_SOURCE = r'''
import base64, collections, json, math, os, signal, subprocess, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Float32, Int32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

NS = '/autodrive/roboracer_1'
CFG = json.loads(base64.b64decode(sys.argv[1]).decode())
STOP = {'flag': False}


def emit(tag, **kw):
    try:
        print(tag + ' ' + json.dumps(kw, default=str), flush=True)
    except (BrokenPipeError, OSError):
        pass


def on_term(signum, frame):
    STOP['flag'] = True


class Mon(Node):
    def __init__(self):
        super().__init__('optimizer_monitor')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)  # matches bridge
        self.qos = qos
        self.lap = self.col = self.last_lap_time = None
        self.speed = self.thr = self.steer = self.front = self.right = None
        self.pose = None  # (x, y, yaw) from odometry
        self.stamp, self.count, self.on_lap = {}, collections.Counter(), None
        self.win = None
        self.create_subscription(Int32, NS + '/lap_count', self.cb_lap, qos)
        self.create_subscription(Float32, NS + '/last_lap_time', self.cb_llt, qos)
        self.create_subscription(Int32, NS + '/collision_count', self.cb_col, qos)
        self.create_subscription(LaserScan, NS + '/lidar', self.cb_scan, qos)
        self.create_subscription(Odometry, NS + '/odom', self.cb_odom, qos)
        self.create_subscription(Float32, NS + '/throttle_command', self.cb_thr, qos)
        self.create_subscription(Float32, NS + '/steering_command', self.cb_steer, qos)
        self.reset_pub = self.create_publisher(Bool, '/autodrive/reset_command', qos)

    def touch(self, name):
        self.stamp[name] = time.monotonic()
        self.count[name] += 1

    def cb_lap(self, m):
        prev, self.lap = self.lap, m.data
        self.touch('lap_count')
        if prev is not None and m.data != prev and self.on_lap:
            self.on_lap(prev, m.data, time.monotonic())

    def cb_llt(self, m):
        self.last_lap_time = float(m.data); self.touch('last_lap_time')

    def cb_col(self, m):
        self.col = m.data; self.touch('collision_count')

    def cb_odom(self, m):
        q, pp = m.pose.pose.orientation, m.pose.pose.position
        self.pose = (pp.x, pp.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        v = m.twist.twist.linear
        self.speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z); self.touch('odom')

    def cb_thr(self, m):
        self.thr = float(m.data); self.touch('throttle_command')

    def cb_steer(self, m):
        self.steer = float(m.data); self.touch('steering_command')

    def cb_scan(self, m):
        self.touch('lidar')
        if self.win is None:
            a = [int(round((s * 0.3491 - m.angle_min) / m.angle_increment)) for s in (-1, 1)]
            self.win = (max(min(a), 0), max(a))
        lo, hi = self.win
        vals = []
        for r in m.ranges[lo:hi + 1]:
            if r == -math.inf:
                vals.append(0.0)
            elif math.isfinite(r) and r > 0.0:
                vals.append(r)
        self.front = min(vals) if vals else 99.0
        i = int(round((-math.pi / 2 - m.angle_min) / m.angle_increment))
        if 0 <= i < len(m.ranges) and math.isfinite(m.ranges[i]):
            self.right = float(m.ranges[i])


def spin(mon, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(mon, timeout_sec=0.02)


def wait_for(mon, cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not STOP['flag']:
        rclpy.spin_once(mon, timeout_sec=0.05)
        if cond():
            return True
    return cond()


def age(mon, name):
    return time.monotonic() - mon.stamp[name] if name in mon.stamp else math.inf


def other_controllers():
    """Any wall_follow_node process other than ours (scans /proc, no ROS needed)."""
    found = []
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            cmd = open('/proc/%s/cmdline' % pid, 'rb').read().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            continue
        if 'wall_follow_node' in cmd and 'of_trial_worker' not in cmd:
            found.append(pid + ' ' + cmd[:140])
    return found


def preflight(mon):
    """Return (ok, info). Read-only: publishes nothing."""
    info = {}
    rival = other_controllers()
    if rival:
        return False, dict(info, reason='preflight: wall_follow_node already running: ' + '; '.join(rival))
    spin(mon, 2.0)  # DDS discovery
    for name in ('lidar', 'lap_count', 'last_lap_time', 'collision_count', 'odom'):
        info['hz_' + name] = None
    t0, n0 = time.monotonic(), dict(mon.count)
    spin(mon, 2.0)
    dt = time.monotonic() - t0
    for name in ('lidar', 'lap_count', 'last_lap_time', 'collision_count', 'odom'):
        info['hz_' + name] = round((mon.count[name] - n0.get(name, 0)) / dt, 1)
    missing = [k[3:] for k, v in info.items() if k.startswith('hz_') and not v]
    if missing:
        return False, dict(info, reason='preflight: no telemetry on ' + ','.join(missing))
    # A killed publisher can linger in discovery for a few seconds.
    def pubs():
        return {t: mon.count_publishers(NS + '/' + t) for t in ('throttle_command', 'steering_command')}
    end = time.monotonic() + 6.0
    while time.monotonic() < end and any(pubs().values()):
        spin(mon, 0.3)
    info['command_publishers'] = pubs()
    info['bridge_subscribers'] = {t: mon.count_subscribers(NS + '/' + t)
                                  for t in ('throttle_command', 'steering_command')}
    info['reset_subscribers'] = mon.count_subscribers('/autodrive/reset_command')
    info.update(lap_count=mon.lap, collisions=mon.col, speed=mon.speed,
                front=mon.front, right=mon.right, last_lap_time=mon.last_lap_time)
    if any(info['command_publishers'].values()):
        return False, dict(info, reason='preflight: another node publishes throttle/steering')
    if not all(info['bridge_subscribers'].values()) or not info['reset_subscribers']:
        return False, dict(info, reason='preflight: bridge not subscribed to commands/reset')
    return True, info


def do_reset(mon):
    for value, duration in ((True, 1.0), (False, 1.0)):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            mon.reset_pub.publish(Bool(data=value))
            spin(mon, 0.1)
    ok = wait_for(mon, lambda: mon.lap == 0 and mon.col == 0, 6.0)
    wait_for(mon, lambda: mon.speed is not None and mon.speed < 0.15, 4.0)
    spin(mon, 0.5)
    return dict(ok=ok and age(mon, 'lidar') < 1.0, lap_count=mon.lap, collisions=mon.col,
                speed=mon.speed)


def parse_log(path):
    reasons, limits, tail = collections.Counter(), collections.Counter(), []
    try:
        lines = open(path, errors='replace').read().splitlines()
    except OSError:
        return reasons, limits, tail
    for line in lines:
        if ']: ' not in line:
            continue
        body = line.split(']: ', 1)[1]
        reason = body.split(': steer=')[0] if ': steer=' in body else None
        if reason and not reason.startswith('tracking'):
            reasons[reason] += 1
        if ' limit=' in body:
            limits[body.rsplit(' limit=', 1)[1].strip()] += 1
    return reasons, limits, lines[-12:]


def stop_controller(proc):
    if proc is None or proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, 6.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        end = time.monotonic() + wait
        while time.monotonic() < end and proc.poll() is None:
            time.sleep(0.1)
        if proc.poll() is not None:
            return


def run_trial(mon):
    c = CFG
    res = dict(status='failed', reason='', laps=[], warmup=[], flying=[], stop_count=0,
               stop_total_s=0.0, max_stop_s=0.0, collisions=0, max_speed=0.0)
    t_begin = time.time()
    ok, info = preflight(mon)
    res['preflight'] = info
    if not ok:
        res['reason'] = info['reason']; return res
    rs = do_reset(mon)
    res['reset'] = rs
    if not rs['ok']:
        res['reason'] = 'reset did not zero lap/collision counters or lidar stale'; return res
    base_lap, base_col = mon.lap, mon.col
    res['start_state'] = dict(front=mon.front, right=mon.right, speed=mon.speed, pose=mon.pose,
                              lap=mon.lap, collisions=mon.col)
    traj, last_traj, still_since = [], 0.0, None
    log_path = '/tmp/of_ctrl_%s.log' % c['trial_id']
    cmd = ['ros2', 'run', 'autodrive_roboracer', 'wall_follow_node', '--ros-args']
    for k, v in c['ros_params']:
        cmd += ['-p', '%s:=%s' % (k, v)]
    res['command'] = ' '.join(cmd)
    proc = None
    ring = collections.deque(maxlen=120)  # ~6 s at 20 Hz
    pending, laps = [], []
    mon.on_lap = lambda prev, new, t: pending.append((t, new, prev))
    t_started = time.monotonic()
    driving_t = stop_since = None
    last_sample = last_progress = t_started
    try:
        proc = subprocess.Popen(cmd, stdout=open(log_path, 'w'), stderr=subprocess.STDOUT,
                                start_new_session=True)
        emit('EVT', event='controller_started', pid=proc.pid)
        # Exactly one publisher must appear on each command topic.
        wait_for(mon, lambda: mon.count_publishers(NS + '/throttle_command') >= 1 and
                 mon.count_publishers(NS + '/steering_command') >= 1, 12.0)
        spin(mon, 0.5)  # discovery settles; a second publisher would show up by now
        npub = (mon.count_publishers(NS + '/throttle_command'),
                mon.count_publishers(NS + '/steering_command'))
        res['command_publishers_running'] = npub
        if npub != (1, 1):
            res['reason'] = 'expected exactly one publisher per command topic, saw %s' % (npub,)
            return res
        t_run = time.monotonic()
        while True:
            rclpy.spin_once(mon, timeout_sec=0.02)
            now = time.monotonic()
            if STOP['flag']:
                res['reason'] = 'interrupted'; break
            if proc.poll() is not None:
                res['reason'] = 'controller_exited rc=%s' % proc.poll(); break
            if age(mon, 'lidar') > 1.0 or age(mon, 'lap_count') > 2.0:
                res['reason'] = 'telemetry_lost'; break
            if mon.col is not None and mon.col > base_col:
                res['collisions'] = mon.col - base_col
                res['reason'] = 'collision'; break
            if mon.speed is not None:
                res['max_speed'] = max(res['max_speed'], mon.speed)
            if now - last_sample >= 0.05:
                last_sample = now
                ring.append((round(now - t_run, 2), mon.speed, mon.thr, mon.steer, mon.front,
                             mon.right, mon.lap, mon.col))
            if now - last_traj >= 0.1:  # whole-trial trajectory, 10 Hz
                last_traj = now
                p = mon.pose or (None, None, None)
                traj.append([None if v is None else round(v, 3) for v in (
                    now - t_run, p[0], p[1], p[2], mon.speed, mon.thr, mon.steer, mon.front,
                    mon.right, mon.lap)])
            thr = mon.thr
            if thr is not None and driving_t is None and thr > 0.05:
                driving_t = now; last_progress = now
            if driving_t is not None and thr is not None:
                if thr <= 1e-6:
                    if stop_since is None:
                        stop_since = now
                    dur = now - stop_since
                    res['max_stop_s'] = max(res['max_stop_s'], dur)
                    if dur > c['max_stop_s']:
                        res['stop_count'] += 1; res['stop_total_s'] += dur
                        res['reason'] = 'safety_stall %.2fs' % dur; break
                elif stop_since is not None:
                    dur = now - stop_since; stop_since = None
                    if dur >= 0.2:
                        res['stop_count'] += 1; res['stop_total_s'] += dur
                        emit('EVT', event='safety_stop', seconds=round(dur, 2))
            if driving_t is not None and mon.speed is not None:
                moved = res['max_speed'] >= 0.5
                if moved and mon.speed < 0.05:
                    still_since = still_since or now
                else:
                    still_since = None
                lingering = moved and still_since is not None and now - still_since >= c['standstill_s']
                never_moved = (not moved) and now - driving_t > c['start_stall_s']
                if lingering or never_moved:
                    thr_now, front_now = mon.thr, mon.front
                    if never_moved:
                        kind = 'start_never_moved'
                    elif thr_now is not None and thr_now <= 1e-6:
                        kind = 'estop_zero_throttle'
                    elif front_now is not None and front_now < 0.6 and (thr_now or 0.0) < 0.02:
                        kind = 'front_brake_deadlock'  # throttle collapsed near the 0.5 m stop distance
                    else:
                        kind = 'standstill_other'
                    t_stop = (still_since or driving_t)
                    res['stall'] = dict(kind=kind, t_s=round(t_stop - t_run, 2), lap_count=mon.lap,
                                        seconds_into_lap=round(t_stop - last_progress, 2), pose=mon.pose,
                                        front=front_now, right=mon.right, throttle=thr_now,
                                        steering=mon.steer, collisions=mon.col - base_col,
                                        lidar_age_s=round(age(mon, 'lidar'), 3),
                                        odom_age_s=round(age(mon, 'odom'), 3))
                    res['reason'] = 'stall: ' + kind
                    break
            while pending and now - pending[0][0] >= 0.3:  # let last_lap_time update
                t_ev, new, prev = pending.pop(0)
                if new != prev + 1:
                    res['reason'] = 'lap_count_jump %s->%s' % (prev, new); pending.clear(); break
                lap_time = mon.last_lap_time
                laps.append(lap_time); last_progress = now
                kind = 'warmup' if len(laps) <= c['warmup_laps'] else 'flying'
                emit('EVT', event='lap', index=len(laps), kind=kind, lap_time=lap_time,
                     collisions=mon.col - base_col, front=mon.front)
                if kind == 'flying' and not (3.0 < (lap_time or 0.0) < 60.0):
                    res['reason'] = 'invalid_lap_time %r' % (lap_time,); break
                if kind == 'flying' and lap_time > c['max_lap_time']:
                    res['status'] = 'slow_abort'
                    res['reason'] = 'flying lap %.2fs exceeds max_lap_time' % lap_time; break
            if res['reason']:
                break
            if len(laps) >= c['warmup_laps'] + c['laps']:
                res['status'] = 'ok'; break
            if now - last_progress > c['lap_timeout']:
                res['reason'] = 'lap_timeout (no lap for %.0fs)' % c['lap_timeout']; break
            if now - t_run > c['trial_timeout']:
                res['reason'] = 'trial_timeout'; break
        if res['status'] != 'ok' and res['status'] != 'slow_abort':
            res['status'] = 'failed'
    finally:
        mon.on_lap = None
        res['laps'] = laps
        res['warmup'] = laps[:c['warmup_laps']]
        res['flying'] = laps[c['warmup_laps']:]
        if mon.col is not None:
            res['collisions'] = max(res['collisions'], mon.col - base_col)
        if stop_since is not None:
            dur = time.monotonic() - stop_since
            res['max_stop_s'] = max(res['max_stop_s'], dur)
        res['driving_started_s'] = None if driving_t is None else round(driving_t - t_started, 2)
        # Evidence snapshot BEFORE the car is stopped/reset.
        recent = [r for r in ring if r[0] >= (ring[-1][0] - 2.0)] if ring else []
        res['evidence'] = dict(
            front_now=mon.front, speed_now=mon.speed, steering_now=mon.steer,
            throttle_now=mon.thr, right_now=mon.right,
            min_front_last2s=min([r[4] for r in recent if r[4] is not None], default=None),
            max_speed_last2s=max([r[1] for r in recent if r[1] is not None], default=None),
            max_abs_steer_last2s=max([abs(r[3]) for r in recent if r[3] is not None], default=None))
        res['ring_last_6s'] = list(ring)
        res['trajectory'] = traj
        res['trajectory_columns'] = ['t_s', 'x', 'y', 'yaw', 'speed', 'throttle_cmd', 'steer_cmd',
                                     'front_min', 'right_90deg', 'lap_count']
        res['ring_columns'] = ['t_s', 'speed', 'throttle_cmd', 'steer_cmd', 'front_min',
                               'right_90deg', 'lap_count', 'collisions']
        stop_controller(proc)
        spin(mon, 0.3)
        reasons, limits, tail = parse_log(log_path)
        res['controller_reasons'] = dict(reasons)
        res['controller_throttle_limits'] = dict(limits)
        res['controller_log_tail'] = tail
        res['duration_s'] = round(time.time() - t_begin, 1)
        try:
            res['post_reset'] = do_reset(mon)  # leave the car at the start line
        except Exception as exc:
            res['post_reset'] = dict(ok=False, error=repr(exc))
    return res


def check_only(mon):
    ok, info = preflight(mon)
    return dict(status='ok' if ok else 'failed', reason=info.get('reason', ''), preflight=info)


def main():
    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGHUP, on_term)
    if CFG.get('mode') != 'check':  # one trial worker per container, ever
        import fcntl
        worker_lock = open('/tmp/of_trial.lock', 'w')
        try:
            fcntl.flock(worker_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            emit('RESULT', status='failed', reason='another optimizer worker is already running in the container',
                 laps=[], warmup=[], flying=[], stop_count=0, stop_total_s=0.0, max_stop_s=0.0,
                 collisions=0)
            return
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    mon = Mon()
    try:
        out = check_only(mon) if CFG.get('mode') == 'check' else run_trial(mon)
    except KeyboardInterrupt:
        out = dict(status='failed', reason='interrupted')
    except Exception as exc:
        import traceback
        out = dict(status='failed', reason='worker exception: ' + repr(exc),
                   laps=[], warmup=[], flying=[], stop_count=0, stop_total_s=0.0,
                   max_stop_s=0.0, collisions=0, traceback=traceback.format_exc())
    emit('RESULT', **out)
    mon.destroy_node()
    rclpy.shutdown()


main()
'''


# --------------------------------------------------------------------------
# Host side
# --------------------------------------------------------------------------
def load_controller():
    spec = importlib.util.spec_from_file_location('wall_follow_ctrl', CONTROLLER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def md5(path):
    return hashlib.md5(Path(path).read_bytes()).hexdigest()


def fmt(name, value, defaults):
    """ROS CLI literal; doubles must contain a decimal point."""
    if isinstance(defaults.get(name), bool):
        return 'true' if value else 'false'
    return repr(round(float(value), 5))


def docker(args, *cmd, inp=None, timeout=60):
    return subprocess.run(['docker', 'exec'] + (['-i'] if inp is not None else []) +
                          [args.container] + list(cmd), input=inp, text=True,
                          capture_output=True, timeout=timeout, encoding='utf-8',
                          errors='replace')


def param_hash(params):
    return json.dumps({k: round(v, SIG_DIGITS) if isinstance(v, float) else v
                       for k, v in sorted(params.items())}, sort_keys=True)


def atomic_write(path, text):
    """Write via a temp file so an interrupt can never leave a truncated file."""
    tmp = path.with_name(path.name + '.tmp')
    with open(tmp, 'w', newline='', encoding='utf-8') as handle:
        handle.write(text)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)


class Study:
    """History + persistence (JSONL is the source of truth; CSV rewritten)."""

    def __init__(self, folder, backup=False):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        # Generated data must never be committed.
        for ignore in {self.folder / '.gitignore', DEFAULT_RESULTS / '.gitignore'}:
            if ignore.parent.exists() and not ignore.exists():
                ignore.write_text('*\n!.gitignore\n')
        self.jsonl = self.folder / 'trials.jsonl'
        self.trials = []
        if backup:  # derived files are regenerated atomically; keep a copy of what existed
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            for name in ('trials.csv', 'summary.json'):
                if (self.folder / name).exists():
                    shutil.copy2(self.folder / name, self.folder / (name + '.bak_' + stamp))
        if self.jsonl.exists():
            for line in self.jsonl.read_text(encoding='utf-8').splitlines():
                if line.strip():
                    self.trials.append(json.loads(line))

    def add(self, record):
        self.trials.append(record)
        with open(self.jsonl, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(record) + '\n')
            handle.flush()
            os.fsync(handle.fileno())
        self.write_csv()
        self.write_summary()

    def write_csv(self):
        names = sorted({k for t in self.trials for k in t['effective']})
        head = ['trial_id', 'stage', 'status', 'valid', 'reason', 'start', 'end', 'duration_s',
                'objective', 'mean_lap', 'fastest_lap', 'warmup_lap', 'lap1', 'lap2', 'lap3',
                'collisions', 'safety_stop_count', 'safety_stop_total_s', 'max_stop_s', 'stall_kind',
                'note']
        with io.StringIO(newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(head + names)
            for t in self.trials:
                flying = t.get('flying') or []
                writer.writerow([
                    t['trial_id'], t['stage'], t['status'], t['valid'], t['reason'], t['start'],
                    t['end'], t['duration_s'], t['objective'], t.get('mean_lap'),
                    t.get('fastest_lap'), (t.get('warmup') or [None])[0],
                    *(flying + [None] * 3)[:3], t['collisions'], t['stop_count'],
                    round(t['stop_total_s'], 2), round(t['max_stop_s'], 2),
                    (t.get('stall') or {}).get('kind'), t.get('note', '')]
                    + [t['effective'].get(n) for n in names])
            atomic_write(self.folder / 'trials.csv', handle.getvalue())

    def groups(self):
        groups = {}
        for t in self.trials:
            g = groups.setdefault(param_hash(t['effective']), dict(
                params=t['effective'], means=[], fastest=[], runs=0, fails=0, ids=[],
                stage=t['stage']))
            g['runs'] += 1
            g['ids'].append(t['trial_id'])
            if t['valid']:
                g['means'].append(t['mean_lap']); g['fastest'].append(t['fastest_lap'])
            else:
                g['fails'] += 1
        for g in groups.values():
            g['n_valid'] = len(g['means'])
            g['mean'] = statistics.mean(g['means']) if g['means'] else None
            g['std'] = statistics.stdev(g['means']) if len(g['means']) > 1 else None
            g['fastest_lap'] = min(g['fastest']) if g['fastest'] else None
        return list(groups.values())

    def baseline_group(self, baseline_effective):
        return next((g for g in self.groups()
                     if param_hash(g['params']) == param_hash(baseline_effective)), None)

    def write_summary(self):
        groups = sorted([g for g in self.groups() if g['mean'] is not None],
                        key=lambda g: g['mean'])
        out = dict(updated=datetime.now().isoformat(timespec='seconds'),
                   trials=len(self.trials),
                   ranking=[dict(mean=g['mean'], std=g['std'], n_valid=g['n_valid'], runs=g['runs'],
                                 fastest_lap=g['fastest_lap'], trial_ids=g['ids'], params=g['params'])
                            for g in groups])
        atomic_write(self.folder / 'summary.json', json.dumps(out, indent=2))


class Searcher:
    def __init__(self, ctrl, args):
        self.args, self.ctrl = args, ctrl
        self.defaults = {f.name: getattr(ctrl_defaults(ctrl), f.name) for f in fields(ctrl.ControllerConfig)}
        try:
            import optuna
            optuna.logging.set_verbosity(optuna.logging.WARNING)
            import warnings
            warnings.filterwarnings('ignore', module='optuna')
            self.optuna = optuna
        except ImportError:
            self.optuna = None
        self.strategy = 'tpe' if args.strategy == 'tpe' and self.optuna else 'random'
        if args.strategy == 'tpe' and not self.optuna:
            print('NOTE: optuna not importable; using seeded random search instead.')

    def effective(self, overrides):
        eff = {k: self.defaults[k] for k in SPACE}
        eff.update({k: v for k, v in BASELINE.items()})
        eff.update(overrides)
        return eff

    def valid(self, overrides):
        try:
            cfg = replace(ctrl_defaults(self.ctrl), **overrides)
            return cfg.adaptive_corner_throttle <= cfg.max_throttle
        except (ValueError, TypeError):
            return False

    def stage_space(self, stage, history):
        space = dict(STAGE_SPACES[stage])
        if stage in ('C', 'D'):
            ok_max = [t['effective']['max_throttle'] for t in history if t['valid']]
            ceiling = min(self.args.throttle_cap,
                          max(ok_max + [BASELINE['max_throttle']]) + self.args.throttle_step)
            space['max_throttle'] = (STAGE_C['max_throttle'][0], max(ceiling, 0.27))
        return space

    def best_valid(self, history, exclude_baseline=True):
        base = param_hash(self.effective({}))
        cands = [t for t in history if t['valid'] and not (
            exclude_baseline and param_hash(t['effective']) == base)]
        return min(cands, key=lambda t: t['mean_lap']) if cands else None

    def suggest(self, stage, history, n):
        rng = random.Random(self.args.seed * 100003 + n)
        space = self.stage_space(stage, history)
        for _ in range(30):
            if stage == 'D':
                best = self.best_valid(history)
                centre = (best or dict(effective=self.effective({})))['effective']
                params = {k: min(max(rng.gauss(centre[k], 0.08 * (hi - lo)), lo), hi)
                          for k, (lo, hi) in space.items()}
            elif self.strategy == 'tpe':
                params = self.tpe(stage, space, history, rng)
            else:
                params = {k: rng.uniform(lo, hi) for k, (lo, hi) in space.items()}
            params = {k: round(v, SIG_DIGITS) for k, v in params.items()}
            if 'adaptive_corner_throttle' in params and 'max_throttle' in params:
                params['adaptive_corner_throttle'] = min(
                    params['adaptive_corner_throttle'], params['max_throttle'])
            if self.valid(dict(BASELINE, **params)):
                return params
        raise RuntimeError('could not draw a valid configuration')

    def tpe(self, stage, space, history, rng):
        optuna = self.optuna
        dists = {k: optuna.distributions.FloatDistribution(lo, hi) for k, (lo, hi) in space.items()}
        sampler = optuna.samplers.TPESampler(
            seed=rng.randrange(2 ** 31), multivariate=True, n_startup_trials=self.args.startup_trials)
        study = optuna.create_study(direction='minimize', sampler=sampler)
        for t in history:
            if t['stage'] not in ('B', 'C', 'D') or t['objective'] is None:
                continue
            p = {k: t['effective'][k] for k in space}
            if all(lo <= p[k] <= hi for k, (lo, hi) in space.items()):
                study.add_trial(optuna.trial.create_trial(
                    params=p, distributions=dists, value=t['objective']))
        return study.ask(fixed_distributions=dists).params


def ctrl_defaults(ctrl):
    """The configuration the ROS node declares as its parameter defaults."""
    tuned = getattr(ctrl, 'TUNED_NODE_DEFAULTS', {})
    return replace(ctrl.ControllerConfig(), **tuned)


def make_plan(args):
    stages = [s for s in args.stages.split(',') if s]
    if args.baseline_only:
        return [('A', args.baseline_repeats)]
    quotas = {}
    a = args.baseline_repeats if 'A' in stages else 0
    rest = max(args.max_trials - a, 0)
    weights = {s: w for s, w in (('B', 0.4), ('C', 0.4), ('D', 0.2)) if s in stages}
    total = sum(weights.values()) or 1.0
    for s in ('B', 'C', 'D'):
        if s in weights:
            quotas[s] = int(round(rest * weights[s] / total))
    drift = rest - sum(quotas.values())
    if quotas:
        quotas[max(quotas, key=quotas.get)] += drift
    plan = [('A', a)] if a else []
    plan += [(s, quotas[s]) for s in ('B', 'C', 'D') if s in quotas and quotas[s] > 0]
    return plan


def next_trial(searcher, study, plan):
    history = study.trials
    for stage, quota in plan:
        done = [t for t in history if t['stage'] == stage]
        if len(done) >= quota:
            continue
        n = len(history)
        if stage == 'A':
            return stage, {}, 'baseline repeat %d/%d' % (len(done) + 1, quota)
        if stage == 'D':
            refine_n = quota // 3
            if len(done) >= refine_n:  # repeats: least-run among top-2 + baseline
                top = sorted([g for g in study.groups() if g['n_valid'] and
                              param_hash(g['params']) != param_hash(searcher.effective({}))],
                             key=lambda g: g['mean'])[:2]
                targets = [g['params'] for g in top] + [searcher.effective({})]
                runs = {param_hash(p): sum(1 for t in history if param_hash(t['effective']) == param_hash(p))
                        for p in targets}
                pick = min(targets, key=lambda p: runs[param_hash(p)])
                keep = {k: pick[k] for k in pick}
                return stage, {k: v for k, v in keep.items() if k in SPACE}, 'repeat of best/baseline'
            return stage, searcher.suggest(stage, history, n), 'local refinement'
        return stage, searcher.suggest(stage, history, n), searcher.strategy
    return None


def build_ros_params(overrides, defaults):
    params = dict(BASELINE)
    params.update(overrides)
    params['log_interval'] = 0.5  # finer failure diagnostics; logging only
    return [(k, fmt(k, v, defaults)) for k, v in params.items()]


LOCK_PATH = DEFAULT_RESULTS / '.optimizer.lock'
STATE = {'pid': None}  # pid (inside the container) of the controller our trial started
COMPETITORS = '[w]all_follow_node|[o]f_trial_worker'


class Blocked(Exception):
    """Another process owns the simulator inputs; nothing was touched."""


class Fatal(Exception):
    """A process we started refused to die; refuse to continue."""


class SingleInstance:
    """Machine-wide single-instance lock held for the life of the process.

    A byte-range lock (msvcrt on Windows, flock elsewhere) on byte 0. The OS
    drops it when the process exits OR is killed, so a leftover lock file never
    blocks anyone: a "stale" file is simply re-locked. Holder details are stored
    at offset 16 so they stay readable while byte 0 is locked.
    """

    def __init__(self, path):
        self.path, self.fd = Path(path), None

    def holder(self):
        try:
            fd = os.open(self.path, os.O_RDONLY | getattr(os, 'O_BINARY', 0))
            try:
                os.lseek(fd, 16, 0)
                raw = os.read(fd, 4096)
            finally:
                os.close(fd)
            return json.loads(raw.decode('utf-8')) if raw.strip(b'\0 \r\n') else None
        except (OSError, ValueError):
            return None

    def acquire(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        previous = self.holder()
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT | getattr(os, 'O_BINARY', 0))
        try:
            os.lseek(fd, 0, 0)
            if msvcrt:
                msvcrt.locking(fd, msvcrt.LK_NBLCK, 1)
            else:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(fd)
            who = previous or {}
            raise SystemExit(
                'Another optimizer instance is already running (pid %s, started %s, command: %s).\n'
                'Only one may drive the simulator. Wait for it, or stop it with Ctrl+C in its terminal.\n'
                'The lock is released automatically if that process dies (%s).' % (
                    who.get('pid', '?'), who.get('started', '?'), ' '.join(who.get('argv', [])) or '?',
                    self.path))
        self.fd = fd
        if previous and previous.get('pid'):
            print('Reclaimed stale optimizer lock left by pid %s (started %s); that process no '
                  'longer holds it.' % (previous.get('pid'), previous.get('started')))
        info = dict(pid=os.getpid(), started=datetime.now().isoformat(timespec='seconds'),
                    argv=sys.argv, host=platform.node())
        os.ftruncate(fd, 16)
        os.lseek(fd, 16, 0)
        os.write(fd, json.dumps(info).encode('utf-8'))
        return self

    def release(self):
        if self.fd is None:
            return
        fd, self.fd = self.fd, None
        try:
            os.ftruncate(fd, 16)  # clear holder details
            os.lseek(fd, 0, 0)
            if msvcrt:
                msvcrt.locking(fd, msvcrt.LK_UNLCK, 1)
            else:
                fcntl.flock(fd, fcntl.LOCK_UN)
        except OSError:
            pass
        finally:
            os.close(fd)


def install_signal_handlers():
    """Turn terminal close / Ctrl+Break / SIGTERM into KeyboardInterrupt so cleanup runs."""
    def handler(signum, frame):
        raise KeyboardInterrupt
    for name in ('SIGTERM', 'SIGBREAK', 'SIGHUP'):
        if hasattr(signal, name):
            try:
                signal.signal(getattr(signal, name), handler)
            except (ValueError, OSError):
                pass


def find_competitors(args):
    """Container processes that could drive the car: any wall_follow_node or trial worker."""
    out = docker(args, 'pgrep', '-fa', COMPETITORS, timeout=30)
    if out.returncode == 1:
        return []
    if out.returncode != 0:
        raise Blocked('could not inspect container processes: ' + out.stderr.strip())
    return [line for line in out.stdout.splitlines() if line.strip()]


def require_clear(args):
    """Refuse (touching nothing) if anything else could be commanding the car."""
    found = find_competitors(args)
    if found:
        raise Blocked(
            'REFUSING TO START: another controller/worker process is running in %s:\n  %s\n'
            'The optimizer will NOT stop it. Stop it yourself (Ctrl+C in the terminal that runs it) '
            'and re-run. Check: docker exec %s pgrep -fa "[w]all_follow_node"' % (
                args.container, '\n  '.join(found), args.container))


def group_alive(args, pgid):
    """True if any non-zombie process still belongs to process group `pgid`."""
    script = "ps -eo pgid=,stat= | awk -v g=%d '$1==g && $2 !~ /^Z/ {f=1} END{exit !f}'" % pgid
    return docker(args, 'bash', '-c', script, timeout=30).returncode == 0


def ensure_controller_gone(args, pid, timeout=20.0):
    """Block until the controller we started has fully exited (escalate on OUR pid only)."""
    if not pid:
        return
    end = time.time() + timeout
    while group_alive(args, pid) and time.time() < end:
        time.sleep(0.5)
    if group_alive(args, pid):
        print('  controller (pid %d) still alive after %.0fs; sending SIGINT then SIGKILL to its '
              'process group' % (pid, timeout), flush=True)
        docker(args, 'bash', '-c', 'kill -INT -- -%d' % pid, timeout=30)
        time.sleep(3)
        if group_alive(args, pid):
            docker(args, 'bash', '-c', 'kill -9 -- -%d' % pid, timeout=30)
            time.sleep(1)
    if group_alive(args, pid):
        raise Fatal('controller process group %d would not exit; not starting another trial' % pid)


def cleanup_container(args, term_wait=12.0):
    """Ctrl+C / timeout / exception path.

    Touches only OUR worker (by name; require_clear guarantees it is the only one)
    and OUR controller (by the pid the worker reported). A controller someone else
    started is never signalled.
    """
    print('Cleaning up: asking the trial worker to stop the controller and reset...', flush=True)
    try:
        docker(args, 'pkill', '-TERM', '-f', '[o]f_trial_worker', timeout=30)
        end = time.time() + term_wait
        while time.time() < end and docker(args, 'pgrep', '-f', '[o]f_trial_worker',
                                           timeout=30).returncode == 0:
            time.sleep(0.5)
        pid = STATE.get('pid')
        if pid and group_alive(args, pid):
            docker(args, 'bash', '-c', 'kill -INT -- -%d' % pid, timeout=30)
            time.sleep(4)
            if group_alive(args, pid):
                docker(args, 'bash', '-c', 'kill -9 -- -%d' % pid, timeout=30)
                time.sleep(1)
        if docker(args, 'pgrep', '-f', '[o]f_trial_worker', timeout=30).returncode == 0:
            docker(args, 'pkill', '-9', '-f', '[o]f_trial_worker', timeout=30)
            time.sleep(1)
        left = find_competitors(args)
        if left:
            print('WARNING: processes still present (not ours to kill?):\n  ' + '\n  '.join(left))
        else:
            print('Cleanup complete: no controller or worker left in the container.')
    except KeyboardInterrupt:
        print('Cleanup interrupted; verify with: docker exec %s pgrep -fa "[w]all_follow_node"'
              % args.container)
    except Exception as exc:
        print('cleanup warning:', exc)
    finally:
        STATE['pid'] = None


def run_worker(args, config, deadline):
    """Stream worker stdout; return the RESULT dict (or a failure dict).

    On a host-side deadline the docker client is killed and the container worker is
    cleaned up explicitly; on any other exception the caller runs cleanup_container.
    """
    payload = base64.b64encode(json.dumps(config).encode()).decode()
    cmd = ['docker', 'exec', args.container, 'bash', '-lc',
           'source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash'
           ' && exec python3 -u %s %s' % (WORKER_PATH, payload)]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            encoding='utf-8', errors='replace')
    expired = {'v': False}

    def expire():
        expired['v'] = True
        proc.kill()

    timer = threading.Timer(deadline, expire)
    timer.daemon = True
    timer.start()
    result, noise = None, []
    try:
        for line in proc.stdout:
            line = line.rstrip()
            if line.startswith('RESULT '):
                result = json.loads(line[7:])
            elif line.startswith('EVT '):
                evt = json.loads(line[4:])
                if evt.get('event') == 'controller_started':
                    STATE['pid'] = evt.get('pid')
                elif evt.get('event') == 'lap':
                    print('    lap %d (%s): %.3f s  collisions=%d  front=%.2f' % (
                        evt['index'], evt['kind'], evt['lap_time'] or float('nan'),
                        evt['collisions'], evt['front'] or float('nan')), flush=True)
                elif evt.get('event') == 'safety_stop':
                    print('    safety stop %.2f s' % evt['seconds'], flush=True)
            else:
                noise.append(line)
        proc.wait()
    except BaseException:
        proc.kill()
        raise
    finally:
        timer.cancel()
    fail = dict(status='failed', laps=[], warmup=[], flying=[], stop_count=0, stop_total_s=0.0,
                max_stop_s=0.0, collisions=0)
    if expired['v']:
        cleanup_container(args)
        return dict(fail, reason='host deadline of %.0fs exceeded; worker and controller cleaned up' % deadline)
    if result is None:
        return dict(fail, reason='worker produced no result (rc=%s): %s' % (
            proc.returncode, ' | '.join(noise[-5:])))
    return result


def load_repeat(args):
    """Exact parameter strings of a saved, valid trial (the -p values that actually ran)."""
    src = Path(args.from_run) / 'trials.jsonl'
    if not src.exists():
        sys.exit('--from-run must name a run folder containing trials.jsonl (missing: %s)' % src)
    recs = [json.loads(line) for line in src.read_text(encoding='utf-8').splitlines() if line.strip()]
    cands = [r for r in recs if r['trial_id'] == args.repeat_trial and r.get('valid') and r.get('ros_command')]
    if len(cands) != 1:
        sys.exit('expected exactly one valid saved trial %d in %s, found %d' % (
            args.repeat_trial, src, len(cands)))
    rec = cands[0]
    toks = rec['ros_command'].split()
    ros = [tuple(toks[i + 1].split(':=', 1)) for i, t in enumerate(toks) if t == '-p']
    problems = []
    for k, v in ros:
        saved = rec['effective'].get(k)
        if isinstance(saved, (int, float)) and not isinstance(saved, bool) and abs(float(v) - saved) > 1e-9:
            problems.append('%s: command %s vs effective %s' % (k, v, saved))
    return rec, ros, problems


def build_repeat_queue(args, searcher):
    rec, ros, problems = load_repeat(args)
    orig = rec.get('mean_lap')
    eff, changes = dict(rec['effective']), []
    for item in args.set or []:  # single-parameter variations around the saved trial
        k, v = item.split('=', 1)
        if k not in SPACE:
            sys.exit('--set: %s is not a searchable parameter (%s)' % (k, ', '.join(sorted(SPACE))))
        eff[k] = round(float(v), 5)
        text = repr(eff[k])
        ros = [(a, text if a == k else b) for a, b in ros] + ([] if any(a == k for a, _ in ros) else [(k, text)])
        changes.append('%s=%s' % (k, text))
    if changes and not searcher.valid({k: v for k, v in eff.items() if k in BASELINE or k in SPACE}):
        sys.exit('--set produces an invalid controller configuration: ' + ', '.join(changes))
    tag = ('trial %d + %s' % (args.repeat_trial, ', '.join(changes))) if changes else 'saved trial %d' % args.repeat_trial
    queue = [('R', eff, 'exact repeat %d/%d of %s (original mean %.3f s)' % (
        i + 1, args.repeat_count, tag, orig), ros) for i in range(args.repeat_count)]
    queue += [('A', searcher.effective({}), 'baseline %d/%d' % (i + 1, args.then_baseline), None)
              for i in range(args.then_baseline)]
    return queue, rec, ros, problems


def classify(result, args, laps_needed):
    flying = [x for x in result.get('flying', []) if x is not None and math.isfinite(x)]
    valid = (result['status'] == 'ok' and len(flying) == laps_needed and
             result['collisions'] == 0 and result['max_stop_s'] <= args.max_stop_s)
    if valid:
        return True, statistics.mean(flying), FAIL_BASE * 0 + statistics.mean(flying)
    missing = laps_needed - len(flying)
    return False, (statistics.mean(flying) if flying else None), FAIL_BASE + missing


def print_table(study, baseline_eff, limit=12):
    groups = sorted([g for g in study.groups() if g['mean'] is not None], key=lambda g: g['mean'])
    base = param_hash(baseline_eff)
    print('\n  rank  mean(s)  std    best lap  valid/runs  trials        notes')
    for i, g in enumerate(groups[:limit], 1):
        std = '%.3f' % g['std'] if g['std'] is not None else '  -  '
        print('  %3d   %.3f   %s   %.3f     %d/%d        %-12s  %s' % (
            i, g['mean'], std, g['fastest_lap'], g['n_valid'], g['runs'],
            ','.join(map(str, g['ids'][:4])), 'BASELINE' if param_hash(g['params']) == base else ''))


def verdict(study, baseline_eff, min_gain):
    base = study.baseline_group(baseline_eff)
    cands = sorted([g for g in study.groups() if g['n_valid'] >= 2 and param_hash(g['params']) != param_hash(baseline_eff) and
                    g['mean'] is not None], key=lambda g: g['mean'])
    if not base or not base['n_valid']:
        return 'No valid baseline runs yet.'
    if not cands:
        return 'No non-baseline configuration has >=2 valid repeat runs; nothing is confirmed.'
    best = cands[0]
    noise = max([x for x in (base['std'], best['std']) if x is not None] + [0.0])
    gain = base['mean'] - best['mean']
    needed = max(min_gain, 2 * noise)
    word = 'CONFIRMED improvement' if gain > needed else 'NOT a confirmed improvement'
    return '%s: best repeated config mean %.3f vs baseline %.3f (gain %.3f s, needed > %.3f).' % (
        word, best['mean'], base['mean'], gain, needed)


def print_dry_run(args, ctrl, searcher, plan, study):
    defaults = ctrl_defaults(ctrl)
    print('=== DRY RUN (no controller started, simulator not reset) ===')
    print('Controller file :', CONTROLLER_PATH)
    print('Repo md5        :', md5(CONTROLLER_PATH))
    try:
        out = docker(args, 'md5sum', CONTAINER_NODE_PATH, timeout=30)
        got = out.stdout.split()[0] if out.returncode == 0 and out.stdout else None
        print('Installed md5   : %s (%s)' % (got, 'MATCHES repo' if got == md5(CONTROLLER_PATH)
                                              else 'DIFFERS from repo - use --sync-controller'))
    except Exception as exc:
        print('Installed md5   : unavailable (%s)' % exc)
    print('\nRuntime parameter defaults declared by the ROS node (verified from the file):')
    for f in fields(ctrl.ControllerConfig):
        raw = getattr(ctrl.ControllerConfig(), f.name)
        eff = getattr(defaults, f.name)
        flag = '' if raw == eff else '   <- node default differs from dataclass default (%r)' % (raw,)
        print('  %-34s %r%s' % (f.name, eff, flag))
    print('\nNOTE: the docstring in wall_follow_node.py says optimization_enabled defaults to false;'
          '\n      that is the dataclass/test default. The ROS node runtime default is %r (TUNED_NODE_DEFAULTS).'
          % defaults.optimization_enabled)
    print('\nSearch strategy:', searcher.strategy, '| seed', args.seed)
    print('Plan (stage: trials):', ', '.join('%s:%d' % p for p in plan))
    for stage in ('B', 'C'):
        print('\nStage %s search ranges:' % stage)
        for k, (lo, hi) in searcher.stage_space(stage, []).items():
            print('  %-28s %.3f .. %.3f' % (k, lo, hi))
    print('\nHard-coded, NOT searchable without a controller change: clearance_factor linear ramp shape,'
          '\n  preview_cap shape, derivative clamp +/-5, dt=0.025 default, 0.2 s lookahead filter,'
          '\n  and the fixed 12 degree / 0.2 steering gates of dynamic lookahead and smoothing.')
    print('\nProposed first trials (illustrative; later ones depend on results):')
    for i, (stage, quota) in enumerate(plan):
        if stage == 'A':
            print('  A: baseline x%d -> %s' % (quota, ' '.join('%s=%s' % kv for kv in build_ros_params({}, searcher.defaults)[:4]) + ' ...'))
            continue
        for j in range(min(2, quota)):
            p = searcher.suggest(stage if stage != 'D' else 'C', [], 1000 * i + j)
            print('  %s: %s' % (stage, ', '.join('%s=%s' % kv for kv in p.items())))
    print('\nPer-trial actions: preflight (telemetry, no competing publishers) -> reset ->'
          '\n  ros2 run autodrive_roboracer wall_follow_node --ros-args -p ... -> 1 warm-up + %d flying laps'
          '\n  -> SIGINT controller -> reset. Existing trials in %s: %d' % (args.laps, study.folder, len(study.trials)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dry-run', action='store_true', help='show the plan; start/reset nothing')
    ap.add_argument('--check-telemetry', action='store_true', help='read-only telemetry/publisher check')
    ap.add_argument('--baseline-only', action='store_true', help='stage A only')
    ap.add_argument('--max-trials', type=int, default=10, help='total trials incl. baseline (default 10)')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--strategy', choices=('tpe', 'random'), default='tpe')
    ap.add_argument('--results-dir', default=str(DEFAULT_RESULTS))
    ap.add_argument('--resume', action='store_true', help='continue the latest (or given) run folder')
    ap.add_argument('--stages', default='A,B,C,D')
    ap.add_argument('--baseline-repeats', type=int, default=2)
    ap.add_argument('--laps', type=int, default=3)
    ap.add_argument('--warmup-laps', type=int, default=1)
    ap.add_argument('--container', default='penn-challenge')
    ap.add_argument('--trial-timeout', type=float, default=110.0)
    ap.add_argument('--lap-timeout', type=float, default=30.0)
    ap.add_argument('--max-lap-time', type=float, default=14.0)
    ap.add_argument('--max-stop-s', type=float, default=0.75, help='longest tolerated safety stop')
    ap.add_argument('--standstill-s', type=float, default=3.0,
                    help='speed < 0.05 for this long after moving = stall (invalid)')
    ap.add_argument('--start-stall-s', type=float, default=8.0,
                    help='never reaching speed 0.5 within this long after throttle starts = stall')
    ap.add_argument('--throttle-cap', type=float, default=0.40)
    ap.add_argument('--throttle-step', type=float, default=0.03)
    ap.add_argument('--startup-trials', type=int, default=4)
    ap.add_argument('--min-gain', type=float, default=0.05)
    ap.add_argument('--repeat-trial', type=int, metavar='N',
                    help='no search: re-run saved valid trial N exactly (needs --from-run)')
    ap.add_argument('--from-run', help='run folder (containing trials.jsonl) holding trial N')
    ap.add_argument('--repeat-count', type=int, default=3, help='repeats of trial N (default 3)')
    ap.add_argument('--then-baseline', type=int, default=0, help='baseline runs queued after the repeats')
    ap.add_argument('--set', action='append', metavar='NAME=VALUE',
                    help='with --repeat-trial: override one searchable parameter (repeatable)')
    ap.add_argument('--stop-on-failure', action='store_true',
                    help='repeat mode: end the session at the first invalid trial')
    ap.add_argument('--self-test', action='store_true',
                    help='test lock + process-management logic; never touches the simulator')
    ap.add_argument('--sync-controller', action='store_true',
                    help='copy the repo controller into the container install dir (no rebuild)')
    args = ap.parse_args()

    ctrl = load_controller()
    searcher = Searcher(ctrl, args)
    plan = make_plan(args)
    root = Path(args.results_dir)
    explicit = args.results_dir != str(DEFAULT_RESULTS)
    new_run = root / ('run_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    has_trials = (root / 'trials.jsonl').exists()
    if args.resume:  # an explicit run folder, else the newest run_* folder
        runs = sorted(p for p in root.glob('run_*') if p.is_dir())
        folder = root if has_trials else (runs[-1] if runs else new_run)
    elif has_trials:
        sys.exit('results dir already holds trials; use --resume or another --results-dir')
    else:
        folder = root if explicit else new_run
    repeat_queue = None
    if args.repeat_trial is not None:
        if not args.from_run:
            sys.exit('--repeat-trial needs --from-run <run folder>')
        repeat_queue, rec, ros, problems = build_repeat_queue(args, searcher)
        if args.resume is False and has_trials:
            sys.exit('results dir already holds trials; use --resume or another --results-dir')
    if args.self_test:
        return self_test(args)
    if args.dry_run:
        study = Study(folder) if folder.exists() else type('S', (), dict(folder=folder, trials=[]))()
        if repeat_queue is not None:
            print('=== DRY RUN: exact repeat (no controller started, simulator not reset) ===')
            print('Source: trial %d of %s (original: mean %.3f s, laps %s)' % (
                args.repeat_trial, args.from_run, rec['mean_lap'], [round(x, 3) for x in rec['flying']]))
            print('Exact -p values that will be passed:\n  ' + '\n  '.join('%s:=%s' % kv for kv in ros))
            for p in problems:
                print('WARNING saved command/effective mismatch:', p)
            print('Queue (%d trials, strictly sequential): %s' % (
                len(repeat_queue), ' -> '.join(q[0] for q in repeat_queue)))
            print('Results would go to:', folder, '(new folder; existing runs are never modified)')
        else:
            print_dry_run(args, ctrl, searcher, plan, study)
        try:
            holder = SingleInstance(LOCK_PATH).holder()
            lock = SingleInstance(LOCK_PATH)
            lock.acquire()
            lock.release()
            print('Single-instance lock: free%s' % (' (stale holder info %s ignored)' % holder if holder else ''))
        except SystemExit as exc:
            print('Single-instance lock: HELD -', exc)
        return 0

    # ---- live run ----------------------------------------------------------
    install_signal_handlers()
    if args.check_telemetry:  # read-only: publishes nothing, so no lock needed
        prepare_container(args)
        result = run_worker(args, dict(mode='check'), 90)
        print(json.dumps(result, indent=2))
        return 0 if result.get('status') == 'ok' else 2
    lock = SingleInstance(LOCK_PATH)
    lock.acquire()
    atexit.register(lock.release)
    try:
        repo_md5 = prepare_container(args)
        require_clear(args)
        return run_session(args, searcher, plan, folder, repeat_queue, repo_md5)
    except Blocked as exc:
        print(exc)
        return 3
    except Fatal as exc:
        print('FATAL:', exc)
        return 4
    finally:
        lock.release()


def prepare_container(args):
    try:
        probe = docker(args, 'true', timeout=30)
    except Exception as exc:
        sys.exit('docker unavailable: %s' % exc)
    if probe.returncode:
        sys.exit('container %s not running: %s' % (args.container, probe.stderr.strip()))
    repo_md5 = md5(CONTROLLER_PATH)
    got = docker(args, 'md5sum', CONTAINER_NODE_PATH)
    installed = got.stdout.split()[0] if got.returncode == 0 and got.stdout else None
    if installed != repo_md5:
        if not args.sync_controller:
            sys.exit('installed controller (%s) differs from repo (%s). Re-run with '
                     '--sync-controller or rebuild the package.' % (installed, repo_md5))
        subprocess.run(['docker', 'cp', str(CONTROLLER_PATH), '%s:%s' % (args.container, CONTAINER_NODE_PATH)],
                       check=True)
        print('Synced repo controller into container.')
    docker(args, 'bash', '-c', 'cat > %s' % WORKER_PATH, inp=WORKER_SOURCE)
    return repo_md5


def run_session(args, searcher, plan, folder, repeat_queue, repo_md5):
    study = Study(folder, backup=True)  # existing CSV/JSON are copied aside, JSONL is append-only
    base_eff = searcher.effective({})
    print('Results folder:', folder)
    if repeat_queue is not None:
        print('Mode: exact repeat, no search | queue of %d: %s | existing trials: %d' % (
            len(repeat_queue), ' -> '.join(q[0] for q in repeat_queue), len(study.trials)))
    else:
        print('Strategy: %s | plan: %s | existing trials: %d' % (
            searcher.strategy, ', '.join('%s:%d' % p for p in plan), len(study.trials)))

    def get_next():
        if repeat_queue is not None:
            i = len(study.trials)
            return repeat_queue[i] if i < len(repeat_queue) else None
        nxt = next_trial(searcher, study, plan)
        if nxt is None:
            return None
        stage, overrides, note = nxt
        return stage, searcher.effective(overrides), note, None

    ran = 0
    try:
        while True:
            nxt = get_next()
            if nxt is None:
                break
            require_clear(args)  # sequential guarantee: nothing else may be commanding the car
            stage, effective, note, ros_params = nxt
            tid = max([t['trial_id'] for t in study.trials], default=0) + 1
            if not searcher.valid({k: v for k, v in effective.items() if k in BASELINE or k in SPACE}):
                print('trial %d invalid configuration, skipped: %s' % (tid, effective))
                study.add(dict(trial_id=tid, stage=stage, status='rejected', valid=False,
                               reason='config validation failed', start=None, end=None,
                               duration_s=0, objective=FAIL_BASE + args.laps, collisions=0,
                               stop_count=0, stop_total_s=0.0, max_stop_s=0.0,
                               effective=effective, note=note))
                continue
            print('\n[trial %d | stage %s | %s]' % (tid, stage, note))
            print('  ' + ', '.join('%s=%s' % (k, effective[k]) for k in sorted(SPACE)))
            config = dict(mode='trial', trial_id=tid, laps=args.laps, warmup_laps=args.warmup_laps,
                          trial_timeout=args.trial_timeout, lap_timeout=args.lap_timeout,
                          max_lap_time=args.max_lap_time, max_stop_s=args.max_stop_s,
                          standstill_s=args.standstill_s, start_stall_s=args.start_stall_s,
                          ros_params=ros_params or build_ros_params(effective, searcher.defaults))
            STATE['pid'] = None
            start = datetime.now()
            result = run_worker(args, config, args.trial_timeout + 120)
            end = datetime.now()
            ensure_controller_gone(args, STATE['pid'])  # fully exited before anything else starts
            STATE['pid'] = None
            valid, mean_lap, objective = classify(result, args, args.laps)
            flying = result.get('flying') or []
            record = dict(
                trial_id=tid, stage=stage, note=note, status=result['status'], valid=valid,
                reason=result.get('reason', ''), start=start.isoformat(timespec='seconds'),
                end=end.isoformat(timespec='seconds'), duration_s=result.get('duration_s'),
                objective=objective, mean_lap=mean_lap if valid else None,
                fastest_lap=min(flying) if valid and flying else None,
                warmup=result.get('warmup'), flying=flying, laps=result.get('laps'),
                collisions=result.get('collisions', 0), stop_count=result.get('stop_count', 0),
                stop_total_s=result.get('stop_total_s', 0.0), max_stop_s=result.get('max_stop_s', 0.0),
                effective=effective, ros_command=result.get('command'), seed=args.seed,
                strategy=searcher.strategy, controller_md5=repo_md5,
                controller_reasons=result.get('controller_reasons'),
                controller_throttle_limits=result.get('controller_throttle_limits'),
                max_speed=result.get('max_speed'), evidence=result.get('evidence'),
                stall=result.get('stall'), start_state=result.get('start_state'))
            if result.get('trajectory'):  # every trial, valid or not, for post-hoc analysis
                (study.folder / ('traj_trial_%04d.json' % tid)).write_text(json.dumps(dict(
                    columns=result.get('trajectory_columns'), rows=result['trajectory'])), encoding='utf-8')
            if not valid:  # keep rich failure telemetry in a side file
                diag = {k: result.get(k) for k in (
                    'reason', 'evidence', 'ring_columns', 'ring_last_6s', 'controller_log_tail',
                    'controller_reasons', 'controller_throttle_limits', 'preflight', 'reset', 'stall',
                    'start_state')}
                (study.folder / ('diag_trial_%04d.json' % tid)).write_text(
                    json.dumps(diag, indent=2), encoding='utf-8')
                record['diag_file'] = 'diag_trial_%04d.json' % tid
            study.add(record)
            ran += 1
            if valid:
                print('  VALID  mean %.3f s | fastest %.3f s | laps %s | stops %d' % (
                    mean_lap, min(flying), ['%.3f' % x for x in flying], record['stop_count']))
            else:
                ev = result.get('evidence') or {}
                if result.get('stall'):
                    st = result['stall']
                    print('  STALL %s at %.1fs (%.1fs into lap, lap_count=%s) pose=%s front=%s right=%s thr=%s' % (
                        st['kind'], st['t_s'], st['seconds_into_lap'], st['lap_count'],
                        [round(v, 2) for v in st['pose']] if st['pose'] else None, st['front'], st['right'],
                        st['throttle']))
                print('  INVALID: %s | collisions=%d stops=%d | front=%s speed=%s' % (
                    record['reason'], record['collisions'], record['stop_count'],
                    ev.get('front_now'), ev.get('speed_now')))
                if record['controller_reasons']:
                    print('  controller reasons:', record['controller_reasons'])
            singles = [t for t in study.trials if t['valid']]
            if singles:
                b = min(singles, key=lambda t: t['mean_lap'])
                print('  best single valid run so far: trial %d mean %.3f s (fastest lap %.3f s)' % (
                    b['trial_id'], b['mean_lap'], min(t['fastest_lap'] for t in singles)))
            if repeat_queue is None and len(study.trials) >= args.max_trials:
                break
            if repeat_queue is not None and args.stop_on_failure and not valid:
                print('  --stop-on-failure: candidate rejected after its first invalid trial.')
                break
    except KeyboardInterrupt:
        print('\nInterrupted: stopping controller and resetting (the in-flight trial is not recorded).')
        cleanup_container(args)
        print('Resume with: --resume --results-dir "%s"' % folder)
        return 130
    except (Blocked, Fatal):
        raise
    except Exception:
        cleanup_container(args)
        raise

    print('\n=== SESSION SUMMARY (%d new trials) ===' % ran)
    print_table(study, base_eff)
    if repeat_queue is not None:
        for g in sorted(study.groups(), key=lambda g: g['stage']):
            if g['n_valid']:
                print('  %s group: %d valid / %d runs, mean %.3f s, std %s, fastest lap %.3f s' % (
                    'baseline' if param_hash(g['params']) == param_hash(base_eff) else 'repeated config',
                    g['n_valid'], g['runs'], g['mean'], '%.3f' % g['std'] if g['std'] is not None else '-',
                    g['fastest_lap']))
    print('\n' + verdict(study, base_eff, args.min_gain))
    print('Files: %s' % folder)
    return 0


def self_test(args):
    """Exercise the lock and process guards. Never resets, publishes, or starts a controller."""
    outcomes = []

    def check(name, ok, detail=''):
        outcomes.append(ok)
        print('%s  %s%s' % ('PASS' if ok else 'FAIL', name, ('  (' + detail + ')') if detail else ''))

    child = (
        "import importlib.util,sys,time\n"
        "sys.dont_write_bytecode=True\n"
        "spec=importlib.util.spec_from_file_location('oc',sys.argv[1]);m=importlib.util.module_from_spec(spec)\n"
        "spec.loader.exec_module(m)\n"
        "try: m.SingleInstance(sys.argv[2]).acquire()\n"
        "except SystemExit: print('DENIED',flush=True); sys.exit(7)\n"
        "print('ACQUIRED',flush=True)\n"
        "if len(sys.argv)>3: time.sleep(120)\n")
    me = str(Path(__file__).resolve())
    tmp = Path(tempfile.mkdtemp(prefix='oc_selftest_')) / 'test.lock'

    def run_child(hold=False):
        cmd = [sys.executable, '-c', child, me, str(tmp)] + (['hold'] if hold else [])
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)

    print('--- single-instance lock (temporary lock file, real lock untouched) ---')
    mine = SingleInstance(tmp).acquire()
    p = run_child()
    out = p.communicate(timeout=60)[0]
    check('second instance is denied while lock is held', p.returncode == 7 and 'DENIED' in out, out.strip())
    mine.release()
    p = run_child()
    out = p.communicate(timeout=60)[0]
    check('lock is free after release', p.returncode == 0 and 'ACQUIRED' in out, out.strip())
    p = run_child(hold=True)
    for line in p.stdout:  # wait until the child really holds the lock
        if 'ACQUIRED' in line:
            break
    denied = None
    try:
        SingleInstance(tmp).acquire()
        denied = False
    except SystemExit:
        denied = True
    check('lock holds while a live process owns it', denied)
    p.kill()
    p.wait()
    time.sleep(0.5)
    try:
        again = SingleInstance(tmp).acquire()
        again.release()
        check('stale lock (holder killed hard) is reclaimed', True)
    except SystemExit as exc:
        check('stale lock (holder killed hard) is reclaimed', False, str(exc)[:80])

    print('--- container process guards (fake process only; no ROS, no simulator) ---')
    try:
        pre = find_competitors(args)
    except Exception as exc:
        check('container reachable', False, str(exc))
        pre = None
    if pre is None:
        pass
    elif pre:
        check('no real controller/worker is running (needed to run these tests safely)', False, pre[0][:80])
    else:
        spawn = ("import subprocess;p=subprocess.Popen(['bash','-c','exec -a wall_follow_node_selftest sleep 300'],"
                 "start_new_session=True,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,"
                 "stderr=subprocess.DEVNULL);print(p.pid)")

        def fake():
            return int(docker(args, 'python3', '-c', spawn, timeout=30).stdout.strip())
        pid = fake()
        time.sleep(1)
        check('competing wall_follow_node process is detected', bool(find_competitors(args)))
        try:
            require_clear(args)
            check('require_clear refuses to start', False)
        except Blocked:
            check('require_clear refuses to start', True)
        check('process group is seen alive', group_alive(args, pid))
        ensure_controller_gone(args, pid, timeout=1.0)
        check('ensure_controller_gone escalates and confirms exit', not group_alive(args, pid))
        check('nothing left after escalation', not find_competitors(args))
        pid2 = fake()
        time.sleep(1)
        STATE['pid'] = pid2
        cleanup_container(args, term_wait=1.0)
        check('cleanup_container kills only the tracked controller pid', not group_alive(args, pid2)
              and not find_competitors(args))

    print('--- exact repeat ---')
    if args.repeat_trial is not None and args.from_run:
        rec, ros, problems = load_repeat(args)
        check('saved trial %d loads with exact -p values' % args.repeat_trial, bool(ros) and not problems,
              '%d params, mean %.3f' % (len(ros), rec['mean_lap']))
    else:
        print('SKIP  (pass --repeat-trial N --from-run <folder> to include)')
    print('\nSELF-TEST %s (%d/%d)' % ('PASSED' if all(outcomes) else 'FAILED', sum(outcomes), len(outcomes)))
    return 0 if all(outcomes) else 1


if __name__ == '__main__':
    sys.exit(main())
