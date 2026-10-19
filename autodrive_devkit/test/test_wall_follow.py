"""Synthetic geometry/safety tests; run with unittest, without ROS or NumPy."""

from dataclasses import replace
import math
from pathlib import Path
import random
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from autodrive_roboracer.wall_follow_node import (  # noqa: E402
    ControllerConfig, ScanView, WallFollower, estimate_wall, stamp_is_fresh,
)


def corridor(right=0.65, left=0.65, slope=0.0, front=10.0, beams=1080,
             angle_min=math.radians(-135), angle_max=math.radians(135)):
    """Ray-cast the lines y=slope*x-right, y=slope*x+left, and x=front."""
    increment = (angle_max - angle_min) / (beams - 1)
    ranges = []
    for index in range(beams):
        angle = angle_min + index * increment
        dx, dy = math.cos(angle), math.sin(angle)
        candidates = [10.0]
        denominator = dy - slope * dx
        if abs(denominator) > 1e-9:
            for intercept in (-right, left):
                distance = intercept / denominator
                if distance > 0:
                    candidates.append(distance)
        if dx > 1e-9:
            candidates.append(front / dx)
        ranges.append(min(candidates))
    return SimpleNamespace(ranges=ranges, angle_min=angle_min,
                           angle_max=angle_max, angle_increment=increment,
                           range_min=0.06, range_max=10.0)


def set_sector(scan, low_deg, high_deg, value):
    """Overwrite only an angular sector, independent of scan resolution."""
    for index in range(len(scan.ranges)):
        angle = math.degrees(scan.angle_min + index * scan.angle_increment)
        if low_deg <= angle <= high_deg:
            scan.ranges[index] = value


class WallFollowerTests(unittest.TestCase):
    """Exercise steering signs, analytic geometry, and fail-stop behavior."""

    def setUp(self):
        self.config = ControllerConfig()

    def settled(self, scan, config=None):
        """Run long enough for throttle acceleration limiting to settle."""
        controller = WallFollower(config or self.config)
        for index in range(100):
            command = controller.update(scan, 1.0 + index * 0.025)
        return command

    def test_parallel_wall_at_target_drives_straight(self):
        command = self.settled(corridor())
        self.assertEqual(command.reason, 'tracking')
        self.assertAlmostEqual(command.steering, 0.0, places=6)
        self.assertAlmostEqual(command.throttle, self.config.max_throttle)

    def test_right_wall_steering_direction(self):
        self.assertGreater(self.settled(corridor(right=0.4)).steering, 0)
        self.assertLess(self.settled(corridor(right=1.3)).steering, 0)

    def test_left_wall_steering_direction(self):
        config = replace(self.config, wall_side='left')
        self.assertLess(self.settled(corridor(left=0.4), config).steering, 0)
        self.assertGreater(self.settled(corridor(left=1.3), config).steering, 0)

    def test_analytic_wall_geometry_and_lookahead(self):
        for side in ('right', 'left'):
            for slope in (-0.25, 0.25):
                with self.subTest(side=side, slope=slope):
                    config = replace(self.config, wall_side=side)
                    scan = corridor(slope=slope)
                    alpha, distance, predicted = estimate_wall(ScanView(scan), config)
                    expected_angle = math.atan(slope) * (1 if side == 'left' else -1)
                    expected_distance = 0.65 / math.sqrt(1 + slope * slope)
                    self.assertAlmostEqual(alpha, expected_angle, places=6)
                    self.assertAlmostEqual(distance, expected_distance, places=6)
                    self.assertAlmostEqual(predicted, expected_distance +
                                           config.lookahead_distance *
                                           math.sin(expected_angle), places=6)

    def test_lookahead_steers_away_from_converging_wall(self):
        # Positive world slope brings the right wall toward the vehicle ahead.
        with_lookahead = self.settled(corridor(slope=0.2))
        without = self.settled(corridor(slope=0.2),
                               replace(self.config, lookahead_distance=0.0))
        self.assertGreater(with_lookahead.steering, without.steering)

    def test_scan_resolution_fov_and_reverse_order(self):
        for beams in (271, 541, 1080, 1081, 1441):
            scan = corridor(right=0.4, beams=beams,
                            angle_min=-2.0, angle_max=2.0)
            forward = self.settled(scan)
            scan.ranges.reverse()
            scan.angle_min, scan.angle_max = scan.angle_max, scan.angle_min
            scan.angle_increment *= -1
            reverse = self.settled(scan)
            self.assertGreater(forward.steering, 0)
            self.assertAlmostEqual(forward.steering, reverse.steering, places=6)

    def test_actual_bridge_endpoint_rounding(self):
        scan = corridor()
        scan.angle_min = -2.35619
        scan.angle_max = 2.35619
        scan.angle_increment = 0.004363323
        self.assertEqual(self.settled(scan).reason, 'tracking')

    def test_throttle_slows_for_corners(self):
        straight = self.settled(corridor())
        corner = self.settled(corridor(right=1.8))
        self.assertLess(corner.throttle, straight.throttle)
        self.assertAlmostEqual(corner.throttle, self.config.min_throttle)

    def test_front_braking_and_stop_override_minimum_throttle(self):
        controller = WallFollower()
        for index in range(50):
            controller.update(corridor(), index * 0.025)
        slow = controller.update(corridor(front=0.7), 1.25)
        self.assertGreater(slow.throttle, 0.0)
        self.assertLess(slow.throttle, self.config.min_throttle)
        stopped = controller.update(corridor(front=0.4), 1.275)
        self.assertEqual(stopped.throttle, 0.0)
        self.assertEqual(stopped.steering, 0.0)
        self.assertEqual(stopped.reason, 'front obstacle')

    def test_single_close_front_beam_is_not_filtered_away(self):
        scan = corridor()
        scan.ranges[round(-scan.angle_min / scan.angle_increment)] = 0.2
        self.assertEqual(self.settled(scan).reason, 'front obstacle')

    def test_positive_infinity_in_front_means_no_return_within_range(self):
        scan = corridor()
        set_sector(scan, -7, 7, math.inf)
        self.assertEqual(self.settled(scan).reason, 'tracking')
        self.assertGreater(self.settled(scan).throttle, 0.0)

    def test_negative_infinity_in_front_means_too_close(self):
        scan = corridor()
        scan.ranges[round(-scan.angle_min / scan.angle_increment)] = -math.inf
        self.assertEqual(self.settled(scan).reason, 'front obstacle')

    def test_invalid_scans_stop(self):
        for value in (math.nan, math.inf, -math.inf, 0.0, -1.0, 0.01, 11.0):
            with self.subTest(value=value):
                scan = corridor()
                scan.ranges = [value] * len(scan.ranges)
                command = self.settled(scan)
                self.assertEqual(command.throttle, 0.0)
                self.assertEqual(command.steering, 0.0)

    def test_missing_wall_and_max_range_are_not_phantom_walls(self):
        for value in (math.nan, math.inf, 10.0, 9.8):
            scan = corridor()
            set_sector(scan, -95, -40, value)
            command = self.settled(scan, replace(
                self.config, allow_opposite_wall_fallback=False))
            self.assertEqual(command.reason, 'wall unavailable')
            self.assertEqual(command.throttle, 0.0)

    def test_opposite_wall_fallback_uses_real_geometry_at_low_throttle(self):
        scan = corridor(left=0.4)
        set_sector(scan, -95, -40, math.inf)
        command = self.settled(scan)
        self.assertEqual(command.reason, 'tracking opposite wall')
        self.assertLess(command.steering, 0.0)
        self.assertLessEqual(command.throttle, self.config.min_throttle)
        set_sector(scan, 40, 95, math.inf)
        self.assertEqual(self.settled(scan).reason, 'wall unavailable')

    def test_switching_reference_wall_clears_derivative(self):
        controller = WallFollower()
        controller.update(corridor(right=0.4), 1.0)
        scan = corridor()
        set_sector(scan, -95, -40, math.inf)
        command = controller.update(scan, 1.025)
        self.assertEqual(command.reason, 'tracking opposite wall')
        self.assertAlmostEqual(command.steering, 0.0, places=6)

    def test_median_rejects_isolated_side_outlier(self):
        scan = corridor()
        index = round((-math.pi / 2 - scan.angle_min) / scan.angle_increment)
        scan.ranges[index] = 0.1
        self.assertAlmostEqual(self.settled(scan).steering, 0.0, places=6)
        scan.ranges[index] = math.nan
        self.assertEqual(self.settled(scan).reason, 'tracking')

    def test_blind_center_sector_stops_even_if_outer_sectors_valid(self):
        scan = corridor()
        set_sector(scan, -7, 7, math.nan)
        command = self.settled(scan)
        self.assertEqual(command.reason, 'insufficient front scan coverage')
        self.assertEqual(command.throttle, 0.0)

    def test_missing_angular_coverage_is_not_edge_clamped(self):
        scan = corridor(angle_min=-0.5, angle_max=0.5)
        self.assertEqual(self.settled(scan).reason, 'wall unavailable')
        scan = corridor(angle_min=-2.0, angle_max=-0.8)
        self.assertEqual(self.settled(scan).reason, 'insufficient front scan coverage')

    def test_malformed_metadata_stops(self):
        cases = {'angle_increment': 0.0, 'angle_min': math.nan,
                 'angle_max': 0.0, 'range_max': -1.0, 'range_min': 11.0,
                 'ranges': []}
        for name, value in cases.items():
            scan = corridor()
            setattr(scan, name, value)
            self.assertEqual(self.settled(scan).throttle, 0.0)

    def test_watchdog_stops_without_another_scan(self):
        controller = WallFollower()
        self.assertEqual(controller.check_timeout(0).throttle, 0)
        controller.update(corridor(), 1.0)
        self.assertIsNone(controller.check_timeout(1.1))
        self.assertEqual(controller.check_timeout(1.31).reason, 'stale scan')
        self.assertEqual(controller.command.throttle, 0)
        self.assertIsNone(controller.previous_error)

    def test_recovery_does_not_reuse_derivative_or_jump_throttle(self):
        controller = WallFollower()
        controller.update(corridor(right=1.5), 1.0)
        controller.check_timeout(2.0)
        recovered = controller.update(corridor(), 2.1)
        self.assertAlmostEqual(recovered.steering, 0, places=6)
        self.assertLessEqual(recovered.throttle, self.config.throttle_rise_rate * 0.025)

    def test_throttle_acceleration_is_limited(self):
        controller = WallFollower()
        first = controller.update(corridor(), 1.0)
        second = controller.update(corridor(), 1.025)
        self.assertAlmostEqual(first.throttle, 0.0125)
        self.assertAlmostEqual(second.throttle, 0.025)

    def test_timestamp_validation(self):
        self.assertTrue(stamp_is_fresh(10.0, 10.05, 9.9, 0.3))
        self.assertFalse(stamp_is_fresh(9.0, 10.0, None, 0.3))
        self.assertFalse(stamp_is_fresh(11.0, 10.0, None, 0.3))
        self.assertFalse(stamp_is_fresh(10.0, 10.05, 10.0, 0.3))
        self.assertFalse(stamp_is_fresh(9.9, 10.05, 10.0, 0.3))
        self.assertTrue(stamp_is_fresh(0.0, 10.0, None, 0.3))
        self.assertFalse(stamp_is_fresh(0.0, 10.0, 9.9, 0.3))

    def test_invalid_configuration_rejected(self):
        for changes in ({'wall_side': 'up'}, {'kp': math.nan}, {'kd': -1.0},
                        {'min_throttle': 0.9, 'max_throttle': 0.2},
                        {'max_throttle': 1.1}, {'scan_timeout': 0.0},
                        {'front_slowdown_distance': 0.3},
                        {'corner_slowdown_threshold': 0.0},
                        {'min_valid_fraction': 0.0}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(self.config, **changes)

    def test_randomized_output_bounds_and_integral_limit(self):
        rng = random.Random(42)
        config = replace(self.config, kp=20.0, ki=3.0, kd=1.0)
        controller = WallFollower(config)
        for index in range(150):
            scan = corridor(right=rng.uniform(0.1, 3.0), left=1.0,
                            slope=rng.uniform(-0.3, 0.3),
                            front=rng.uniform(0.2, 10.0), beams=541)
            command = controller.update(scan, 1.0 + index * 0.025)
            self.assertTrue(math.isfinite(command.steering))
            self.assertTrue(math.isfinite(command.throttle))
            self.assertLessEqual(abs(command.steering), 1.0)
            self.assertTrue(0.0 <= command.throttle <= config.max_throttle)
            self.assertLessEqual(abs(controller.integral), config.integral_limit)


if __name__ == '__main__':
    unittest.main()
