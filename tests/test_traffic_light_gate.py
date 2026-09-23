#!/usr/bin/env python3
"""Focused decision tests for the non-bypassable stop-line gate."""

import math
import pathlib
import sys
import time
import unittest
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))

from geometry_msgs.msg import Twist

from cmd_vel_watchdog import (CmdVelWatchdog, START_LINE, STOP_LINES,
                              ZEBRA_PIXEL_BOXES, DRIVE_AXLE_OFFSET)


class TrafficGateTest(unittest.TestCase):
    def setUp(self):
        self.gate = CmdVelWatchdog.__new__(CmdVelWatchdog)
        self.gate.pose = None
        self.gate.pose_wall = time.monotonic()
        self.gate.enforce_traffic = True
        self.gate.white_map = np.zeros((4200, 4200), dtype=bool)
        self.gate.detected = 'RED'
        self.gate.detected_count = 3
        self.gate.detected_wall = time.monotonic()
        self.gate.sim_state = 'RED'
        self.gate.green_remaining = 10.0
        self.gate.committed = {name: False for name, *_ in STOP_LINES}
        self.gate.passed = {name: False for name, *_ in STOP_LINES}
        self.command = Twist()
        self.command.linear.x = 0.25

    def decision(self, pose):
        self.gate.pose = pose
        return self.gate.gate_command(self.command, time.monotonic())

    def set_green(self, remaining=10.0):
        self.gate.detected = 'GREEN'
        self.gate.detected_count = 3
        self.gate.detected_wall = time.monotonic()
        self.gate.sim_state = 'GREEN'
        self.gate.green_remaining = remaining

    def test_red_stops_before_westbound_line(self):
        command, status = self.decision((-0.375, 0.00, math.pi))
        self.assertEqual(command.linear.x, 0.0)
        self.assertEqual(status, 'WESTBOUND:WAIT_RED')

    def test_yellow_stops_before_northbound_line(self):
        self.gate.detected = self.gate.sim_state = 'YELLOW'
        command, status = self.decision((1.70, -0.430, math.pi / 2))
        self.assertEqual(command.linear.x, 0.0)
        self.assertEqual(status, 'NORTHBOUND:WAIT_YELLOW')

    def test_zebra_is_allowed_only_during_stable_green(self):
        row_min, row_max, col_min, col_max = ZEBRA_PIXEL_BOXES[0]
        row, col = (row_min + row_max) // 2, (col_min + col_max) // 2
        self.assertFalse(self.gate.permitted_white_pixel(0.0, 0.0, row, col, time.monotonic()))
        self.set_green()
        self.assertTrue(self.gate.permitted_white_pixel(0.0, 0.0, row, col, time.monotonic()))

    def test_start_line_is_always_traversable_in_its_own_lane(self):
        axis, line, lateral_center, _half_width, _thickness = START_LINE
        x, y = (lateral_center, line) if axis == 'y' else (line, lateral_center)
        self.assertTrue(self.gate.permitted_white_pixel(x, y, 0, 0, time.monotonic()))

    def test_red_stops_translation_but_keeps_in_place_turn(self):
        self.command.angular.z = 0.6
        command, status = self.decision((-0.375, 0.00, math.pi))
        self.assertEqual(command.linear.x, 0.0)
        self.assertEqual(command.angular.z, 0.6)
        self.assertEqual(status, 'WESTBOUND:WAIT_RED')

    def test_missing_or_stale_detection_fails_safe(self):
        self.gate.detected = 'GREEN'
        self.gate.sim_state = 'GREEN'
        self.gate.detected_wall = time.monotonic() - 2.0
        command, status = self.decision((-0.375, 0.00, math.pi))
        self.assertEqual(command.linear.x, 0.0)
        self.assertTrue(status.startswith('WESTBOUND:WAIT_'))

    def test_stable_green_admits_crossing(self):
        self.set_green()
        command, status = self.decision((-0.375, 0.00, math.pi))
        self.assertEqual(command.linear.x, 0.25)
        self.assertEqual(status, 'WESTBOUND:GO_GREEN')
        self.assertTrue(self.gate.committed['WESTBOUND'])

    def test_ending_green_does_not_admit(self):
        self.set_green(remaining=1.5)
        command, status = self.decision((-0.375, 0.00, math.pi))
        self.assertEqual(command.linear.x, 0.0)
        self.assertEqual(status, 'WESTBOUND:WAIT_GREEN_ENDING')

    def test_admitted_vehicle_clears_during_yellow(self):
        self.gate.committed['WESTBOUND'] = True
        self.gate.detected = self.gate.sim_state = 'YELLOW'
        command, status = self.decision((-0.535, 0.00, math.pi))
        self.assertEqual(command.linear.x, 0.25)
        self.assertEqual(status, 'WESTBOUND:CLEARING')

    def test_idle_green_does_not_carry_authorization_into_red(self):
        self.set_green()
        stopped = Twist()
        self.gate.pose = (-0.375, 0.00, math.pi)
        _, status = self.gate.gate_command(stopped, time.monotonic())
        self.assertEqual(status, 'WESTBOUND:READY_GREEN')
        self.assertFalse(self.gate.committed['WESTBOUND'])
        self.gate.detected = self.gate.sim_state = 'RED'
        command, status = self.gate.gate_command(self.command, time.monotonic())
        self.assertEqual(command.linear.x, 0.0)
        self.assertEqual(status, 'WESTBOUND:WAIT_RED')

    def test_authorization_revoked_if_light_changes_before_line(self):
        self.set_green()
        self.decision((-0.375, 0.00, math.pi))
        self.assertTrue(self.gate.committed['WESTBOUND'])
        self.gate.detected = self.gate.sim_state = 'RED'
        command, status = self.decision((-0.375, 0.00, math.pi))
        self.assertEqual(command.linear.x, 0.0)
        self.assertFalse(self.gate.committed['WESTBOUND'])
        self.assertEqual(status, 'WESTBOUND:WAIT_RED')

    def test_approach_speed_is_limited(self):
        command, status = self.decision((-0.205, 0.00, math.pi))
        self.assertEqual(command.linear.x, 0.12)
        self.assertEqual(status, 'WESTBOUND:APPROACH')

    def test_ignored_signals_only_exempt_measured_stop_line(self):
        self.gate.enforce_traffic = False
        self.assertTrue(self.gate.permitted_stop_line_pixel(-0.515, 0.0))
        self.assertFalse(self.gate.permitted_stop_line_pixel(0.5, 0.5))
        self.assertFalse(self.gate.permitted_white_pixel(0.5, 0.5, 10, 10, time.monotonic()))

    def test_stable_green_never_exempts_ordinary_paint(self):
        self.set_green()
        self.gate.committed = {name: True for name, *_ in STOP_LINES}
        self.assertFalse(self.gate.permitted_white_pixel(0.5, 0.5, 10, 10, time.monotonic()))

    def test_full_polygon_catches_thin_paint_between_old_samples(self):
        self.gate.enforce_traffic = False
        row, col = self.gate.map_pixel(0.501, 0.501, 4200)
        self.gate.white_map[row, col] = True
        self.assertEqual(self.gate.footprint_white_status(0.5, 0.5, 0, time.monotonic()),
                         'WHITE_LINE:BLOCKED')

    def test_exact_positive_footprint_edge_is_checked(self):
        self.gate.enforce_traffic = False
        row, col = self.gate.map_pixel(0.594, 0.585, 4200)
        self.gate.white_map[row, col] = True
        self.assertEqual(self.gate.footprint_white_status(0.5, 0.5, 0, time.monotonic()),
                         'WHITE_LINE:BLOCKED')

    def test_missing_and_stale_pose_stop_even_when_signals_ignored(self):
        self.gate.enforce_traffic = False
        command, status = self.gate.gate_command(self.command, time.monotonic())
        self.assertEqual(command.linear.x, 0.0)
        self.assertEqual(status, 'WHITE_LINE:POSE_UNAVAILABLE')
        self.gate.pose = (0.5, 0.5, 0.0)
        self.gate.pose_wall = time.monotonic() - 2.0
        command, status = self.gate.gate_command(self.command, time.monotonic())
        self.assertEqual(command.linear.x, 0.0)
        self.assertEqual(status, 'WHITE_LINE:POSE_STALE')
        self.assertEqual(self.gate.white_line_status(time.monotonic(), self.command), status)

    def test_rotation_is_about_drive_axle(self):
        x, y, yaw = self.gate.predicted_pose(0, 0, 0, 0, 1, math.pi / 2)
        self.assertAlmostEqual(x, DRIVE_AXLE_OFFSET)
        self.assertAlmostEqual(y, -DRIVE_AXLE_OFFSET)
        self.assertAlmostEqual(yaw, math.pi / 2)

    def test_curved_motion_uses_arc(self):
        x, y, yaw = self.gate.predicted_pose(0, 0, 0, 0.2, 1, math.pi / 2)
        self.assertAlmostEqual(x, 0.2 + DRIVE_AXLE_OFFSET)
        self.assertAlmostEqual(y, 0.2 - DRIVE_AXLE_OFFSET)


if __name__ == '__main__':
    unittest.main()
