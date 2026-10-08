from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from wheel_odometry import (integrate_encoder_pose, signed_velocity,
                            validate_yaw_scale)


class WheelOdometryTest(unittest.TestCase):
    def test_scale_one_preserves_the_encoder_pose(self):
        previous_raw = (0.0, 0.0, 0.0)
        current_raw = (0.7, 0.2, 0.8)
        self.assertEqual(
            integrate_encoder_pose(previous_raw, current_raw, previous_raw, 1.0),
            current_raw)

    def test_yaw_scale_corrects_rotation_and_forward_arc(self):
        raw = (math.cos(0.5), math.sin(0.5), 1.0)
        corrected = integrate_encoder_pose((0.0, 0.0, 0.0), raw,
                                           (0.0, 0.0, 0.0), 0.96)
        self.assertAlmostEqual(corrected[0], math.cos(0.48), places=6)
        self.assertAlmostEqual(corrected[1], math.sin(0.48), places=6)
        self.assertAlmostEqual(corrected[2], 0.96, places=6)

    def test_yaw_scale_handles_wrapped_encoder_angle(self):
        previous_raw = (0.0, 0.0, math.radians(170.0))
        current_raw = (0.0, 0.0, math.radians(-170.0))
        previous_corrected = (0.0, 0.0, math.radians(170.0))
        corrected = integrate_encoder_pose(previous_raw, current_raw,
                                           previous_corrected, 0.95)
        expected = math.radians(170.0) + math.radians(20.0) * 0.95
        self.assertAlmostEqual(corrected[2], expected, places=6)

    def test_invalid_yaw_scale_is_rejected(self):
        for scale in (0.0, -0.1, 1.6, float('nan')):
            with self.subTest(scale=scale):
                with self.assertRaises(ValueError):
                    validate_yaw_scale(scale)

    def test_signed_velocity_uses_scaled_pose_samples(self):
        previous = (0.0, 0.0, 0.0)
        current = (0.0, 0.0, 0.48)
        linear, angular = signed_velocity(previous, current, 2.0)
        self.assertAlmostEqual(linear, 0.0)
        self.assertAlmostEqual(angular, 0.24)


if __name__ == '__main__':
    unittest.main()
