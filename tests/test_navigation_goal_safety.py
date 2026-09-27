"""Geometric regressions for goals beside walls and white-line cells."""
import math
import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from navigation_goal_safety import GridFootprintChecker, quaternion_yaw


class GoalSafetyTests(unittest.TestCase):
    def checker(self, cells=(), **kwargs):
        data = [0] * 10000
        for col, row, value in cells:
            data[row * 100 + col] = value
        return GridFootprintChecker(100, 100, 0.01, data, **kwargs)

    def test_rear_overhang_rejected_when_center_is_clear(self):
        result = self.checker([(36, 50, 100)], safety_margin=0).check_pose(.5, .5, 0)
        self.assertFalse(result.safe)
        self.assertEqual(result.cell, (36, 50))

    def test_heading_changes_which_side_contains_rear_overhang(self):
        checker = self.checker([(36, 50, 100)], safety_margin=0)
        self.assertTrue(checker.check_pose(.5, .5, math.pi).safe)
        self.assertFalse(checker.check_pose(.5, .5, 0).safe)

    def test_cell_wholly_inside_polygon_is_not_missed(self):
        self.assertFalse(self.checker([(46, 50, 100)], safety_margin=0)
                         .check_pose(.5, .5, 0).safe)

    def test_margin_checks_cell_area_not_just_cell_center(self):
        # Front edge .542, occupied square starts .57: gap .028 metres.
        checker = self.checker([(57, 50, 100)])
        self.assertFalse(checker.check_pose(.5, .5, 0).safe)
        self.assertTrue(self.checker([(59, 50, 100)]).check_pose(.5, .5, 0).safe)

    def test_unknown_is_rejected_soft_cost_is_not_a_wall(self):
        self.assertFalse(self.checker([(46, 50, -1)]).check_pose(.5, .5, 0).safe)
        self.assertTrue(self.checker([(46, 50, 99)]).check_pose(.5, .5, 0).safe)

    def test_narrow_corridor_retains_valid_goal(self):
        walls = [(c, r, 100) for c in range(100) for r in (34, 65)]
        self.assertTrue(self.checker(walls).check_pose(.5, .5, 0).safe)
        self.assertFalse(self.checker(walls).check_pose(.5, .5, math.pi / 2).safe)

    def test_rotated_map_origin(self):
        grid = self.checker([(36, 50, 100)], origin=(1, 2, math.pi / 2))
        self.assertFalse(grid.check_pose(.5, 2.5, math.pi / 2).safe)
        self.assertTrue(grid.check_pose(.5, 2.5, -math.pi / 2).safe)

    def test_outside_map_and_nonfinite_goal_rejected(self):
        grid = self.checker()
        self.assertFalse(grid.check_pose(.05, .5, 0).safe)
        self.assertFalse(grid.check_pose(float('nan'), .5, 0).safe)

    def test_touching_cell_is_collision(self):
        tiny = ((.04, .02), (.04, -.02), (-.04, -.02), (-.04, .02))
        grid = self.checker([(54, 50, 100)], footprint=tiny, safety_margin=0)
        self.assertFalse(grid.check_pose(.5, .5, 0).safe)

    def test_invalid_quaternion_rejected(self):
        with self.assertRaises(ValueError):
            quaternion_yaw(SimpleNamespace(x=0, y=0, z=0, w=0))


if __name__ == '__main__':
    unittest.main()
