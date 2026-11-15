"""Dependency-free helpers for the TeleAvatar left-arm hold mode."""

from __future__ import annotations

import numpy as np


def capture_left_pose(measured_arm14: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    """Return a safe, independent seven-joint snapshot of the measured left arm."""
    measured_arm14 = np.asarray(measured_arm14, dtype=np.float64)
    lower = np.asarray(lower, dtype=np.float64)
    upper = np.asarray(upper, dtype=np.float64)
    if measured_arm14.shape != (14,) or lower.shape != (7,) or upper.shape != (7,):
        raise ValueError("expected measured_arm14=(14,), lower=(7,), upper=(7,)")
    if not np.all(np.isfinite(measured_arm14)):
        raise ValueError("measured arm pose contains a non-finite value")
    return np.clip(measured_arm14[:7], lower, upper).copy()


def apply_left_pose(arm14: np.ndarray, frozen_left7: np.ndarray) -> np.ndarray:
    """Copy an arm target and replace only its left seven joints."""
    arm14 = np.asarray(arm14, dtype=np.float64)
    frozen_left7 = np.asarray(frozen_left7, dtype=np.float64)
    if arm14.shape != (14,) or frozen_left7.shape != (7,):
        raise ValueError("expected arm14=(14,) and frozen_left7=(7,)")
    held = arm14.copy()
    held[:7] = frozen_left7
    return held
