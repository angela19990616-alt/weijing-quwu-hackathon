import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / "robot"))
from ta2_common import DIRECTION_COMMANDS, validate_duration, validate_velocity


class TA2ControlHelpersTest(unittest.TestCase):
    def test_measured_right_mapping(self):
        self.assertEqual(DIRECTION_COMMANDS["right"], (0.05, 0.0, 0.0))

    def test_velocity_limits(self):
        self.assertEqual(validate_velocity(0.03, 0.04, 0.2), (0.03, 0.04, 0.2))
        with self.assertRaises(ValueError):
            validate_velocity(0.05, 0.05, 0.0)
        with self.assertRaises(ValueError):
            validate_velocity(float("nan"), 0.0, 0.0)

    def test_duration_limits(self):
        self.assertEqual(validate_duration(2), 2.0)
        for invalid in (0, -1, 5.1, float("inf")):
            with self.assertRaises(ValueError):
                validate_duration(invalid)


if __name__ == "__main__":
    unittest.main()
