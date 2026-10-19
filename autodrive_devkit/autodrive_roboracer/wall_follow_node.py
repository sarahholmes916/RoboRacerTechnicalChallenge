#!/usr/bin/env python3
"""Geometric wall following with a ROS-independent, synthetic-testable core.

Angles use the LaserScan convention: x forward, y left, radians. Commands are
normalized AutoDRIVE inputs, NOT steering radians or a speed in metres/second.

Performance features are opt-in: optimization_enabled:=true. With it false,
the original controller is used, including its fixed lookahead and throttle law.
Start with throttle alone: leave dynamic_lookahead_enabled false and
steering_smoothing_tau zero. Tune adaptive_corner_throttle in 0.01 increments,
then max_throttle in 0.01 increments, using repeated three-clean-lap trials.
Reject collisions, safety stops, and inconsistent laps, not just slow best laps.
Only then evaluate dynamic lookahead or smoothing, one at a time. Target wall
distance remains fixed: a racing-line change would confound these comparisons.

All parameters are startup-only. To restore the original settings, restart with
no parameter overrides (optimization_enabled defaults to false). Safety stops,
invalid-data handling, and reliable command publishers apply in both modes.
"""

from dataclasses import dataclass, fields, replace
import math
import signal
import time
from typing import Optional, Tuple

# Allow the controller and tests to run on a host without ROS installed. Missing
# ROS dependencies other than rclpy itself still raise their normal import error.
try:
    import rclpy
except ModuleNotFoundError as exc:
    if exc.name != 'rclpy':
        raise
    rclpy = None

if rclpy is not None:
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32
else:
    Node = object


def clamp(value: float, low: float, high: float) -> float:
    """Constrain a finite value to an inclusive interval."""
    return max(low, min(high, value))


@dataclass(frozen=True)
class ControllerConfig:
    """Startup parameters; distances are metres and times are seconds."""

    wall_side: str = 'right'
    allow_opposite_wall_fallback: bool = True
    desired_wall_distance: float = 0.65
    lookahead_distance: float = 0.8
    kp: float = 1.2
    ki: float = 0.0
    kd: float = 0.08
    min_throttle: float = 0.12
    max_throttle: float = 0.26
    corner_slowdown_threshold: float = 0.35
    front_obstacle_safety_distance: float = 0.5
    front_slowdown_distance: float = 1.7
    front_sector_half_angle_deg: float = 20.0
    wall_ray_separation_deg: float = 45.0
    sample_half_width_deg: float = 2.0
    max_wall_range: float = 9.5
    min_valid_fraction: float = 0.6
    scan_timeout: float = 0.3
    derivative_filter_tau: float = 0.1
    integral_limit: float = 0.5
    throttle_rise_rate: float = 0.5
    log_interval: float = 2.0
    # Independently test speed first; leave steering changes disabled initially.
    optimization_enabled: bool = False
    adaptive_corner_throttle: float = 0.12
    straight_steering_threshold: float = 0.0
    preview_half_angle_deg: float = 40.0
    preview_path_half_width: float = 0.3
    preview_slowdown_distance: float = 2.5
    dynamic_lookahead_enabled: bool = False
    lookahead_extension: float = 0.15
    lookahead_max: float = 1.0
    steering_smoothing_tau: float = 0.0
    steering_smoothing_max_delta: float = 0.08

    def __post_init__(self):
        """Reject unsafe or nonsensical configuration before driving."""
        if self.wall_side not in ('right', 'left'):
            raise ValueError('wall_side must be right or left')
        boolean_fields = ('allow_opposite_wall_fallback', 'optimization_enabled',
                          'dynamic_lookahead_enabled')
        for field in fields(self):
            if field.name not in ('wall_side',) + boolean_fields:
                value = getattr(self, field.name)
                if not math.isfinite(value) or value < 0.0:
                    raise ValueError(field.name + ' must be finite and >= 0')
        for name in ('desired_wall_distance', 'max_wall_range', 'scan_timeout',
                     'throttle_rise_rate', 'log_interval',
                     'front_obstacle_safety_distance'):
            if getattr(self, name) <= 0.0:
                raise ValueError(name + ' must be > 0')
        if not 0.0 <= self.min_throttle <= self.max_throttle <= 1.0:
            raise ValueError('require 0 <= min_throttle <= max_throttle <= 1')
        if not 0.0 < self.corner_slowdown_threshold <= 1.0:
            raise ValueError('corner_slowdown_threshold must be in (0, 1]')
        if not 0.0 < self.min_valid_fraction <= 1.0:
            raise ValueError('min_valid_fraction must be in (0, 1]')
        if self.front_slowdown_distance <= self.front_obstacle_safety_distance:
            raise ValueError('front_slowdown_distance must exceed safety distance')
        if not 5.0 <= self.front_sector_half_angle_deg <= 45.0:
            raise ValueError('front_sector_half_angle_deg must be in [5, 45]')
        if not 20.0 <= self.wall_ray_separation_deg <= 70.0:
            raise ValueError('wall_ray_separation_deg must be in [20, 70]')
        if not 0.0 < self.sample_half_width_deg <= 5.0:
            raise ValueError('sample_half_width_deg must be in (0, 5]')
        if self.max_wall_range <= self.desired_wall_distance:
            raise ValueError('max_wall_range must exceed desired_wall_distance')
        for name in boolean_fields:
            if not isinstance(getattr(self, name), bool):
                raise ValueError(name + ' must be a boolean')
        if not 0.0 <= self.adaptive_corner_throttle <= 1.0:
            raise ValueError('adaptive_corner_throttle must be in [0, 1]')
        if self.optimization_enabled and self.adaptive_corner_throttle > self.max_throttle:
            raise ValueError('adaptive_corner_throttle must not exceed max_throttle')
        if (self.optimization_enabled and not
                0.0 <= self.straight_steering_threshold < self.corner_slowdown_threshold):
            raise ValueError('straight_steering_threshold must be below corner threshold')
        if not 20.0 <= self.preview_half_angle_deg <= 60.0:
            raise ValueError('preview_half_angle_deg must be in [20, 60]')
        if self.preview_path_half_width <= 0.0 or (
                self.optimization_enabled and
                self.preview_path_half_width >= self.desired_wall_distance):
            raise ValueError('preview_path_half_width must be between 0 and wall distance')
        if self.preview_slowdown_distance <= self.front_obstacle_safety_distance:
            raise ValueError('preview_slowdown_distance must exceed safety distance')
        if self.dynamic_lookahead_enabled and self.lookahead_max < self.lookahead_distance:
            raise ValueError('lookahead_max must be >= lookahead_distance')
        if not 0.0 <= self.steering_smoothing_tau <= 0.15:
            raise ValueError('steering_smoothing_tau must be in [0, 0.15] seconds')
        if not 0.0 <= self.steering_smoothing_max_delta <= 0.2:
            raise ValueError('steering_smoothing_max_delta must be in [0, 0.2]')


@dataclass(frozen=True)
class Command:
    """Actuator commands and diagnostics from one controller step."""

    steering: float = 0.0
    throttle: float = 0.0
    reason: str = 'waiting for scan'
    wall_distance: Optional[float] = None
    predicted_distance: Optional[float] = None
    wall_angle: Optional[float] = None
    front_clearance: Optional[float] = None
    preview_clearance: Optional[float] = None
    lookahead_used: Optional[float] = None
    raw_steering: float = 0.0
    throttle_limit: str = 'stop'


class ScanView:
    """Select angular sectors using metadata, including reversed scans."""

    def __init__(self, scan):
        self.scan = scan
        values = (scan.angle_min, scan.angle_max, scan.angle_increment,
                  scan.range_min, scan.range_max)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('non-finite scan metadata')
        if len(scan.ranges) < 2 or abs(scan.angle_increment) < 1e-8:
            raise ValueError('empty scan or zero angle increment')
        if not 0.0 <= scan.range_min < scan.range_max:
            raise ValueError('invalid scan range limits')
        end = scan.angle_min + (len(scan.ranges) - 1) * scan.angle_increment
        # The bridge advertises +135 degrees but its last beam is +134.75.
        if abs(end - scan.angle_max) > 1.5 * abs(scan.angle_increment):
            raise ValueError('scan angles disagree with range count')
        self.low, self.high = sorted((scan.angle_min, end))

    def sector(self, low: float, high: float):
        """Return (range, actual angle) pairs, or [] if coverage is absent."""
        tolerance = abs(self.scan.angle_increment)
        if low < self.low - tolerance or high > self.high + tolerance:
            return []
        start, end = sorted(((low - self.scan.angle_min) /
                             self.scan.angle_increment,
                             (high - self.scan.angle_min) /
                             self.scan.angle_increment))
        first = max(0, math.ceil(start - 1e-6))
        last = min(len(self.scan.ranges) - 1, math.floor(end + 1e-6))
        return [(self.scan.ranges[i], self.scan.angle_min +
                 i * self.scan.angle_increment) for i in range(first, last + 1)]

    def valid(self, distance: float) -> bool:
        """Reject NaN, infinity, zero, and out-of-range returns."""
        return (math.isfinite(distance) and distance > 0.0 and
                self.scan.range_min <= distance <= self.scan.range_max)

    def wall_sample(self, angle: float, config: ControllerConfig):
        """Select a median return and retain its actual beam angle."""
        half_width = math.radians(config.sample_half_width_deg)
        samples = self.sector(angle - half_width, angle + half_width)
        valid = sorted((distance, beam) for distance, beam in samples
                       if self.valid(distance) and distance <
                       min(config.max_wall_range, self.scan.range_max * 0.995))
        if not samples or len(valid) / len(samples) < config.min_valid_fraction:
            return None
        return valid[len(valid) // 2]

    def front_clearance(self, config: ControllerConfig) -> Optional[float]:
        """Use the closest return, requiring coverage in all three sub-sectors.

        AutoDRIVE uses REP-117 positive infinity for no return within range.
        This is clear to range_max in front, but never a wall measurement.
        Negative infinity indicates a too-close return and forces a stop.
        """
        half = math.radians(config.front_sector_half_angle_deg)
        nearest = self.scan.range_max
        for low, high in ((-half, -half / 3), (-half / 3, half / 3),
                          (half / 3, half)):
            samples = self.sector(low, high)
            valid = []
            for distance, _ in samples:
                if distance == math.inf:
                    valid.append(self.scan.range_max)
                elif distance == -math.inf:
                    valid.append(0.0)
                elif self.valid(distance):
                    valid.append(distance)
            if not samples or len(valid) / len(samples) < config.min_valid_fraction:
                return None
            nearest = min(nearest, min(valid))
        return nearest

    def path_clearance(self, config: ControllerConfig) -> Optional[float]:
        """Preview a straight corridor using multiple rays over a wider fan.

        Side walls outside the corridor do not masquerade as frontal obstacles.
        This is an additional speed cap, not a replacement for the original
        front-cone minimum or a prediction of the full curved vehicle footprint.
        Require coverage in each third of the fan; unknown coverage forbids any
        speed increase over the original throttle calculation.
        """
        half = math.radians(config.preview_half_angle_deg)
        nearest = self.scan.range_max
        for low, high in ((-half, -half / 3), (-half / 3, half / 3), (half / 3, half)):
            samples = self.sector(low, high)
            known = 0
            for distance, angle in samples:
                if distance == -math.inf:
                    return 0.0
                if distance == math.inf:
                    known += 1
                elif self.valid(distance):
                    known += 1
                    x = distance * math.cos(angle)
                    y = distance * math.sin(angle)
                    if abs(y) <= config.preview_path_half_width:
                        nearest = min(nearest, x)
            if not samples or known / len(samples) < config.min_valid_fraction:
                return None
        return nearest


def estimate_wall(view: ScanView, config: ControllerConfig
                  ) -> Optional[Tuple[float, float, float]]:
    """Return (angle, perpendicular distance, lookahead distance) to the wall.

    Reflect left/right into coordinates with positive y toward the chosen wall.
    The usual F1TENTH formula is alpha = atan2(a*cos(theta)-b, a*sin(theta)).
    Cartesian endpoints generalize it to the actual median-selected ray angles.
    Positive alpha means the wall recedes from the car ahead.
    """
    side = 1.0 if config.wall_side == 'left' else -1.0
    b = view.wall_sample(side * math.pi / 2, config)
    a = view.wall_sample(side * math.radians(
        90.0 - config.wall_ray_separation_deg), config)
    if a is None or b is None:
        return None
    ax, ay = a[0] * math.cos(a[1]), side * a[0] * math.sin(a[1])
    bx, by = b[0] * math.cos(b[1]), side * b[0] * math.sin(b[1])
    if ax - bx <= 1e-6:
        return None
    alpha = math.atan2(ay - by, ax - bx)
    distance = by * math.cos(alpha) - bx * math.sin(alpha)
    if distance <= 0.0:
        return None
    predicted = distance + config.lookahead_distance * math.sin(alpha)
    return alpha, distance, predicted


class WallFollower:
    """PD/PID controller with fail-stop scan handling and bounded commands."""

    def __init__(self, config: Optional[ControllerConfig] = None):
        self.config = config or ControllerConfig()
        self.opposite_config = replace(
            self.config, wall_side='left' if self.config.wall_side == 'right' else 'right')
        self.active_side = None
        self.last_scan_time = None
        self.command = Command()
        self.previous_error = None
        self.integral = 0.0
        self.derivative = 0.0
        self.filtered_lookahead = self.config.lookahead_distance

    def stop(self, reason: str, front: Optional[float] = None) -> Command:
        """Stop immediately and clear PID history before any recovery."""
        self.previous_error = None
        self.integral = 0.0
        self.derivative = 0.0
        self.command = Command(reason=reason, front_clearance=front)
        self.active_side = None
        self.filtered_lookahead = self.config.lookahead_distance
        return self.command

    def check_timeout(self, now: float) -> Optional[Command]:
        """Watchdog uses monotonic wall time, independent of simulation time."""
        if self.last_scan_time is None:
            return self.stop('waiting for scan')
        age = now - self.last_scan_time
        if not math.isfinite(age) or age < 0 or age > self.config.scan_timeout:
            return self.stop('stale scan')
        return None

    def update(self, scan, now: float) -> Command:
        """Consume a LaserScan-like object and a monotonic receipt time."""
        config = self.config
        if not math.isfinite(now):
            return self.stop('invalid receipt time')
        dt = 0.025 if self.last_scan_time is None else now - self.last_scan_time
        self.last_scan_time = now
        if dt <= 0 or dt > config.scan_timeout:
            self.stop('scan timing reset')
            dt = 0.025
        dt = max(dt, 0.001)
        try:
            view = ScanView(scan)
        except ValueError as exc:
            return self.stop('invalid scan: ' + str(exc))
        front = view.front_clearance(config)
        if front is None:
            return self.stop('insufficient front scan coverage')
        if front <= config.front_obstacle_safety_distance:
            return self.stop('front obstacle', front)
        wall = estimate_wall(view, config)
        wall_config = config
        if wall is None and config.allow_opposite_wall_fallback:
            wall_config = self.opposite_config
            wall = estimate_wall(view, wall_config)
        if wall is None:
            return self.stop('wall unavailable', front)
        alpha, distance, predicted = wall
        fallback = wall_config.wall_side != config.wall_side
        switched_wall = self.active_side != wall_config.wall_side
        preview = view.path_clearance(config) if config.optimization_enabled else None
        if preview is not None and preview <= 0.0:
            return self.stop('preview below sensor range', front)
        lookahead = config.lookahead_distance
        if config.optimization_enabled and config.dynamic_lookahead_enabled:
            # Throttle fraction is a pace proxy, NOT measured speed. Extend only
            # in a clear, nearly parallel corridor; return immediately to the
            # proven lookahead for bends, braking, fallback, or a reference change.
            clear_straight = (not fallback and not switched_wall and preview is not None
                              and preview >= config.preview_slowdown_distance
                              and front >= config.front_slowdown_distance
                              and abs(alpha) < math.radians(12.0)
                              and abs(self.command.raw_steering) < 0.2)
            if clear_straight:
                pace = self.command.throttle / max(config.max_throttle, 1e-6)
                alignment = 1.0 - abs(alpha) / math.radians(12.0)
                target = min(config.lookahead_max, config.lookahead_distance +
                             config.lookahead_extension * pace * alignment)
                weight = dt / (0.2 + dt)
                self.filtered_lookahead += weight * (target - self.filtered_lookahead)
            else:
                self.filtered_lookahead = config.lookahead_distance
            lookahead = self.filtered_lookahead
            predicted = distance + lookahead * math.sin(alpha)
        if switched_wall:
            # Changing the reference wall changes the error discontinuously.
            self.previous_error = None
            self.integral = 0.0
            self.derivative = 0.0
            self.active_side = wall_config.wall_side
        side = 1.0 if wall_config.wall_side == 'left' else -1.0
        # Right wall too close => positive error => LEFT steering, away from it.
        error = side * (predicted - config.desired_wall_distance)
        if self.previous_error is not None:
            raw_derivative = clamp((error - self.previous_error) / dt, -5.0, 5.0)
            weight = dt / (config.derivative_filter_tau + dt)
            self.derivative += weight * (raw_derivative - self.derivative)
        self.previous_error = error
        candidate = clamp(self.integral + error * dt,
                          -config.integral_limit, config.integral_limit)
        effort = config.kp * error + config.ki * candidate + config.kd * self.derivative
        # Integrate only if unsaturated or if the error unwinds saturation.
        if abs(effort) <= 1.0 or effort * error < 0.0:
            self.integral = candidate
        steering = clamp(config.kp * error + config.ki * self.integral +
                         config.kd * self.derivative, -1.0, 1.0)
        raw_steering = steering
        if (config.optimization_enabled and config.steering_smoothing_tau > 0.0
                and not switched_wall and not fallback and preview is not None
                and preview >= config.preview_slowdown_distance
                and front >= config.front_slowdown_distance
                and abs(raw_steering) < 0.2
                and abs(raw_steering - self.command.steering) <=
                config.steering_smoothing_max_delta):
            # Large corrections, bends, and all stops bypass smoothing. Speed
            # selection uses raw demand so filtering cannot conceal a sharp turn.
            weight = dt / (config.steering_smoothing_tau + dt)
            steering = self.command.steering + weight * (
                raw_steering - self.command.steering)
        corner = min(1.0, abs(steering) / config.corner_slowdown_threshold)
        throttle = config.max_throttle - corner * (
            config.max_throttle - config.min_throttle)
        if fallback:
            throttle = min(throttle, config.min_throttle)
        clearance_factor = clamp(
            (front - config.front_obstacle_safety_distance) /
            (config.front_slowdown_distance - config.front_obstacle_safety_distance),
            0.0, 1.0)
        # Safety may go BELOW min_throttle; never impose a minimum during braking.
        throttle *= clearance_factor
        limit = 'original'
        if config.optimization_enabled and preview is not None:
            demand = max(abs(raw_steering), abs(steering))
            corner = clamp((demand - config.straight_steering_threshold) /
                           (config.corner_slowdown_threshold -
                            config.straight_steering_threshold), 0.0, 1.0)
            target = config.max_throttle - corner * (
                config.max_throttle - config.adaptive_corner_throttle)
            # Keep the existing conservative fallback speed even when raising
            # corner throttle. Keep the original front braking factor as well.
            if fallback:
                target = min(target, config.min_throttle)
            target *= clearance_factor
            limit = 'corner' if corner >= 1.0 else 'steering'
            if clearance_factor < 1.0:
                limit = 'front'
            preview_factor = clamp(
                (preview - config.front_obstacle_safety_distance) /
                (config.preview_slowdown_distance - config.front_obstacle_safety_distance),
                0.0, 1.0)
            # A straight corridor is a useful anticipation model BEFORE a turn,
            # but not the car's swept path once steering is large. Fade only this
            # additional cap as corner demand grows; the original front minimum,
            # braking factor, and emergency stop remain active at every angle.
            preview_cap = config.max_throttle * (
                preview_factor + corner * (1.0 - preview_factor))
            throttle = min(target, preview_cap)
            if preview_cap < target:
                limit = 'preview'
            if fallback:
                limit = 'opposite wall'
        elif config.optimization_enabled:
            limit = 'preview unknown: original cap'
        if throttle > self.command.throttle + config.throttle_rise_rate * dt:
            limit = 'acceleration'
        throttle = min(throttle, self.command.throttle + config.throttle_rise_rate * dt)
        self.command = Command(
            steering=steering, throttle=clamp(throttle, 0.0, config.max_throttle),
            reason='tracking opposite wall' if fallback else 'tracking',
            wall_distance=distance, predicted_distance=predicted,
            wall_angle=alpha, front_clearance=front, preview_clearance=preview,
            lookahead_used=lookahead, raw_steering=raw_steering, throttle_limit=limit)
        return self.command


def stamp_is_fresh(stamp: float, ros_now: float, previous: Optional[float],
                   timeout: float) -> bool:
    """Reject old/future/replayed stamps; allow unstamped scans until stamped."""
    if not math.isfinite(stamp) or not math.isfinite(ros_now) or stamp < 0:
        return False
    if stamp == 0.0:
        return previous is None
    return abs(ros_now - stamp) <= timeout and (previous is None or stamp > previous)


TUNED_NODE_DEFAULTS = dict(optimization_enabled=True, adaptive_corner_throttle=0.14)


class WallFollowNode(Node):
    """ROS 2 adapter for the controller, sensor subscription, and watchdog."""

    def __init__(self):
        if rclpy is None:
            raise RuntimeError('ROS 2 is unavailable; source /opt/ros/humble/setup.bash')
        super().__init__('wall_follow_node')
        # Node defaults are the simulator-verified tuned setup; the dataclass
        # defaults stay original so the synthetic baseline remains testable.
        defaults = replace(ControllerConfig(), **TUNED_NODE_DEFAULTS)
        values = {}
        descriptions = {
            'optimization_enabled': 'Enable adaptive throttle and forward preview; false '
                                    'bypasses all new performance features.',
            'adaptive_corner_throttle': 'Normalized corner throttle before front braking. '
                                        'Start at 0.12; evaluate 0.01 increments.',
            'straight_steering_threshold': 'Normalized steering below which full cruise '
                                           'throttle is requested. Zero preserves the '
                                           'original linear steering slowdown curve.',
            'preview_half_angle_deg': 'Half-width in degrees of the multi-ray preview fan.',
            'preview_path_half_width': 'Half-width in metres of the forward corridor; '
                                       'include vehicle width and a clearance margin.',
            'preview_slowdown_distance': 'Metres at which forward preview starts to cap '
                                         'throttle before a turn; larger brakes earlier.',
            'dynamic_lookahead_enabled': 'Optional clear-straight lookahead extension; '
                                         'test only after tuning throttle.',
            'lookahead_extension': 'Maximum added lookahead in metres, scaled by throttle '
                                   'fraction and wall alignment. Throttle is not speed.',
            'lookahead_max': 'Absolute maximum dynamic lookahead in metres. Bends and '
                            'fallback immediately restore lookahead_distance.',
            'steering_smoothing_tau': 'Seconds of filtering for small clear-straight '
                                      'corrections only. Zero disables; large corrections '
                                      'and safety stops always bypass filtering.',
            'steering_smoothing_max_delta': 'Largest normalized steering change that may '
                                            'be smoothed; larger corrections bypass it.',
        }
        for field in fields(defaults):
            descriptor = ParameterDescriptor(
                read_only=True, description=descriptions.get(field.name, '') +
                ' Startup setting; restart node to change.')
            values[field.name] = self.declare_parameter(
                field.name, getattr(defaults, field.name), descriptor).value
        self.controller = WallFollower(ControllerConfig(**values))
        self.last_stamp = None
        self.last_log_time = -math.inf
        # Bridge command subscribers are RELIABLE; BEST_EFFORT publishers would
        # not connect. The scan subscriber accepts reliable or best-effort sensors.
        commands_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        scan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.throttle_pub = self.create_publisher(
            Float32, '/autodrive/roboracer_1/throttle_command', commands_qos)
        self.steering_pub = self.create_publisher(
            Float32, '/autodrive/roboracer_1/steering_command', commands_qos)
        self.scan_sub = self.create_subscription(
            LaserScan, '/autodrive/roboracer_1/lidar', self.scan_callback, scan_qos)
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.watchdog = self.create_timer(
            min(0.05, self.controller.config.scan_timeout / 2.0),
            self.watchdog_callback, clock=self.watchdog_clock)
        self.publish_command(self.controller.stop('waiting for scan'))
        self.get_logger().info('Wall follower ready: ' + str(self.controller.config))

    def publish_command(self, command: Command):
        """Apply final finite/bounds checks at the actuator boundary."""
        steering, throttle = command.steering, command.throttle
        if not math.isfinite(steering) or not math.isfinite(throttle):
            steering, throttle = 0.0, 0.0
        self.steering_pub.publish(Float32(data=float(clamp(steering, -1.0, 1.0))))
        self.throttle_pub.publish(Float32(data=float(clamp(throttle, -1.0, 1.0))))

    def log_command(self, command: Command, now: float):
        """Throttle diagnostics by wall time, even with a paused simulation."""
        if now - self.last_log_time < self.controller.config.log_interval:
            return
        self.last_log_time = now
        message = (f'{command.reason}: steer={command.steering:+.3f} '
                   f'throttle={command.throttle:.3f} '
                   f'wall={command.wall_distance} predicted={command.predicted_distance} '
                   f'front={command.front_clearance} preview={command.preview_clearance} '
                   f'lookahead={command.lookahead_used} raw_steer={command.raw_steering:+.3f} '
                   f'limit={command.throttle_limit}')
        if command.reason.startswith('tracking'):
            self.get_logger().info(message)
        else:
            self.get_logger().warning(message)

    def scan_callback(self, scan):
        """Reject stale headers before refreshing the receipt-time watchdog."""
        now = time.monotonic()
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        if not stamp_is_fresh(stamp, ros_now, self.last_stamp,
                              self.controller.config.scan_timeout):
            command = self.controller.stop('stale or out-of-order scan stamp')
        else:
            if stamp != 0.0:
                self.last_stamp = stamp
            command = self.controller.update(scan, now)
        self.publish_command(command)
        self.log_command(command, now)

    def watchdog_callback(self):
        """Publish zero repeatedly if no fresh scan arrives."""
        now = time.monotonic()
        command = self.controller.check_timeout(now)
        if command is not None:
            self.publish_command(command)
            self.log_command(command, now)

    def stop_vehicle(self):
        """Publish a best-effort shutdown stop while the ROS context is alive."""
        self.publish_command(self.controller.stop('shutdown'))


def main(args=None):
    """Initialize, spin, and stop on Ctrl+C/SIGTERM before destroying ROS."""
    if rclpy is None:
        raise RuntimeError('ROS 2 is unavailable; source /opt/ros/humble/setup.bash')
    # Keep the context alive long enough to publish zeros on normal termination.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    previous_handlers = {}

    def request_stop(signum, frame):
        raise KeyboardInterrupt

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[sig] = signal.signal(sig, request_stop)
        node = WallFollowNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                if rclpy.ok():
                    for _ in range(3):
                        node.stop_vehicle()
                        time.sleep(0.03)
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)


if __name__ == '__main__':
    main()
