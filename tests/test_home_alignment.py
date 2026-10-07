from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from home_alignment import (HOME_XY_TOLERANCE_M, HOME_YAW_TOLERANCE_RAD,
                            home_navigation_tolerances,
                            should_retry_home_alignment)


class HomeAlignmentPolicyTest(unittest.TestCase):
    def test_home_navigation_uses_parking_tolerances(self):
        self.assertEqual(home_navigation_tolerances(.04, .05),
                         (HOME_XY_TOLERANCE_M, HOME_YAW_TOLERANCE_RAD))

    def test_stricter_existing_settings_are_preserved(self):
        self.assertEqual(home_navigation_tolerances(.02, .03), (.02, .03))

    def test_retry_is_limited_to_successful_pose_mismatch(self):
        self.assertTrue(should_retry_home_alignment('ARRIVAL_MISMATCH', True, 0))
        self.assertFalse(should_retry_home_alignment('ARRIVAL_MISMATCH', False, 0))
        self.assertFalse(should_retry_home_alignment('NO_ROUTE_PROGRESS', True, 0))
        self.assertFalse(should_retry_home_alignment('ARRIVAL_MISMATCH', True, 1))

    def test_invalid_tolerance_is_rejected(self):
        with self.assertRaises(ValueError):
            home_navigation_tolerances(float('nan'), .04)


if __name__ == '__main__':
    unittest.main()
