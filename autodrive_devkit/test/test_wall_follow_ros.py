"""Opt-in real ROS messaging/shutdown test on an isolated ROS_DOMAIN_ID.

Run after colcon build with WALL_FOLLOW_ROS_TESTS=1 ROS_DOMAIN_ID=87. The test
publishes synthetic scans and commands, so never use the simulator's domain.
"""

import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

from test_wall_follow import corridor
from autodrive_roboracer.wall_follow_node import rclpy

if rclpy is not None:
    from ament_index_python.packages import get_package_prefix
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32


@unittest.skipUnless(rclpy is not None and os.environ.get('WALL_FOLLOW_ROS_TESTS') == '1',
                     'requires ROS and explicit isolated-domain opt-in')
class RosAdapterTests(unittest.TestCase):
    """Exercise the installed executable through DDS with a frozen ROS clock."""

    def test_commands_watchdog_invalid_scan_and_sigterm(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '87',
                         'this test is restricted to unused test domain 87')
        rclpy.init()
        probe = rclpy.create_node('wall_follow_test_probe')
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        publisher = probe.create_publisher(
            LaserScan, '/autodrive/roboracer_1/lidar', reliable)
        throttle, steering = [], []
        subscriptions = [
            probe.create_subscription(
                Float32, '/autodrive/roboracer_1/throttle_command',
                lambda msg: throttle.append(msg.data), reliable),
            probe.create_subscription(
                Float32, '/autodrive/roboracer_1/steering_command',
                lambda msg: steering.append(msg.data), reliable),
        ]
        executable = (Path(get_package_prefix('autodrive_roboracer')) / 'lib' /
                      'autodrive_roboracer' / 'wall_follow_node')
        # No /clock publisher: this explicitly tests the steady-time watchdog
        # while simulated time is frozen at zero.
        process = subprocess.Popen(
            [str(executable), '--ros-args', '-p', 'use_sim_time:=true'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

        def pump(duration, scan=None):
            deadline = time.monotonic() + duration
            next_publish = 0.0
            while time.monotonic() < deadline:
                if scan is not None and time.monotonic() >= next_publish:
                    publisher.publish(scan)
                    next_publish = time.monotonic() + 0.025
                rclpy.spin_once(probe, timeout_sec=0.01)

        try:
            deadline = time.monotonic() + 8.0
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                pump(0.05)
                self.assertIsNone(process.poll(), 'controller exited during startup')
            self.assertGreater(publisher.get_subscription_count(), 0)
            synthetic = corridor(right=0.4)
            scan = LaserScan()
            for name in ('ranges', 'angle_min', 'angle_max', 'angle_increment',
                         'range_min', 'range_max'):
                setattr(scan, name, getattr(synthetic, name))
            pump(0.8, scan)
            self.assertTrue(throttle and steering, 'no command messages received')
            self.assertGreater(throttle[-1], 0)
            self.assertGreater(steering[-1], 0, 'close right wall must steer left')
            pump(0.5)
            self.assertEqual(throttle[-1], 0, 'watchdog failed with paused ROS clock')
            self.assertEqual(steering[-1], 0)
            scan.ranges = [float('nan')] * len(scan.ranges)
            pump(0.15, scan)
            self.assertEqual(throttle[-1], 0)
            scan.ranges = synthetic.ranges
            pump(0.4, scan)
            self.assertGreater(throttle[-1], 0, 'valid scans should permit recovery')
            process.send_signal(signal.SIGTERM)
            pump(0.3)
            output, _ = process.communicate(timeout=5)
            self.assertEqual(process.returncode, 0, output)
            self.assertEqual(throttle[-1], 0, 'SIGTERM must publish a stop')
            self.assertEqual(steering[-1], 0)
            self.assertTrue(all(0 <= value <= 1 for value in throttle))
            self.assertTrue(all(-1 <= value <= 1 for value in steering))
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.communicate(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate(timeout=5)
            for subscription in subscriptions:
                probe.destroy_subscription(subscription)
            probe.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
