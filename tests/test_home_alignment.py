from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from home_alignment import (HOME_AMCL_REFRESH_INTERVAL_S,
                            HOME_XY_TOLERANCE_M, HOME_YAW_TOLERANCE_RAD,
                            amcl_sample_is_newer,
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

    def test_amcl_refresh_requires_a_newer_finite_sample(self):
        self.assertTrue(amcl_sample_is_newer(10.0, 10.1))
        self.assertFalse(amcl_sample_is_newer(10.0, 10.0))
        self.assertFalse(amcl_sample_is_newer(10.0, 9.9))
        self.assertFalse(amcl_sample_is_newer(10.0, float('nan')))

    def test_amcl_refresh_retries_at_a_bounded_interval(self):
        self.assertGreater(HOME_AMCL_REFRESH_INTERVAL_S, 0.0)
        self.assertLessEqual(HOME_AMCL_REFRESH_INTERVAL_S, 0.5)


if __name__ == '__main__':
    unittest.main()
