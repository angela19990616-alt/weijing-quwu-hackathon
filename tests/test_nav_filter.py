import math
import pathlib
import sys
import unittest

import numpy as np


sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / "robot" / "navigation" / "b70" / "nav"))
from nav_filter import PoseEKF, encoder_discontinuity, wrap_angle


class PoseEKFTest(unittest.TestCase):
    def test_body_prediction_uses_heading(self):
        ekf = PoseEKF([0, 0, math.pi / 2])
        ekf.predict([1, 0, 0])
        np.testing.assert_allclose(ekf.x[:2], [0, 1], atol=1e-9)

    def test_waypoint_correction_reduces_error_and_uncertainty(self):
        ekf = PoseEKF([0, 0, 0], np.diag([0.5, 0.5, 0.5]))
        before = np.trace(ekf.P)
        ekf.update_waypoint(1.0, 2.0, 0.1, position_std_m=0.03)
        self.assertLess(np.linalg.norm(ekf.x[:2] - [1, 2]), 0.02)
        self.assertLess(np.trace(ekf.P), before)

    def test_heading_residual_wraps(self):
        ekf = PoseEKF([0, 0, math.radians(179)], np.diag([1, 1, 1]))
        ekf.update_waypoint(0, 0, math.radians(-179))
        self.assertLess(abs(wrap_angle(ekf.x[2] - math.pi)), math.radians(3))

    def test_encoder_jump_detection(self):
        self.assertFalse(encoder_discontinuity(np.array([0.001, 0.002, 0.001]), 0.01))
        self.assertTrue(encoder_discontinuity(np.array([0.5, 0.0, 0.0]), 0.01))
        self.assertTrue(encoder_discontinuity(np.zeros(3), 0.8))


if __name__ == "__main__":
    unittest.main()
