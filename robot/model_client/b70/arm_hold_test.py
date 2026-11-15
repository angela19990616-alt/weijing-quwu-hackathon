import numpy as np
import pytest

from arm_hold import apply_left_pose, capture_left_pose


def test_capture_clamps_once_and_returns_independent_snapshot():
    measured = np.arange(14, dtype=np.float64)
    lower = np.full(7, 1.0)
    upper = np.full(7, 5.0)

    frozen = capture_left_pose(measured, lower, upper)
    measured[:7] = -99.0

    np.testing.assert_array_equal(frozen, [1.0, 1.0, 2.0, 3.0, 4.0, 5.0, 5.0])


def test_apply_replaces_left_and_preserves_right_without_mutating_policy_action():
    action = np.arange(14, dtype=np.float64)
    original = action.copy()
    frozen = np.full(7, 0.25)

    held = apply_left_pose(action, frozen)

    np.testing.assert_array_equal(held[:7], frozen)
    np.testing.assert_array_equal(held[7:], original[7:])
    np.testing.assert_array_equal(action, original)


@pytest.mark.parametrize("bad_shape", [(13,), (15,)])
def test_capture_rejects_wrong_arm_shape(bad_shape):
    with pytest.raises(ValueError):
        capture_left_pose(np.zeros(bad_shape), np.zeros(7), np.ones(7))
