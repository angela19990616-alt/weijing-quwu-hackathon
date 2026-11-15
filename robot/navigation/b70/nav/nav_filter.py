"""Small EKF for the TA2 fixed-heading waypoint map.

The prediction comes from calibrated wheel odometry.  Absolute corrections
come from parking on a surveyed waypoint and pressing the controller X button.
Without an absolute observation this filter can represent uncertainty, but it
cannot remove systematic wheel slip or long-term drift.
"""

from __future__ import annotations

import math

import numpy as np


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def encoder_discontinuity(body_delta: np.ndarray, dt: float) -> bool:
    """Reject reconnect/reset jumps that cannot be one physical sample."""
    body_delta = np.asarray(body_delta, dtype=float)
    if body_delta.shape != (3,) or not np.all(np.isfinite(body_delta)):
        return True
    if not math.isfinite(dt) or dt <= 0 or dt > 0.5:
        return True
    # Deliberately above the controller limits, yet far below a reboot jump.
    return math.hypot(*body_delta[:2]) > 0.03 or abs(body_delta[2]) > 0.12


class PoseEKF:
    """State [map-right, map-forward, heading] with a 3x3 covariance."""

    def __init__(self, pose, covariance=None):
        self.x = np.asarray(pose, dtype=float).copy()
        if self.x.shape != (3,) or not np.all(np.isfinite(self.x)):
            raise ValueError("pose must contain three finite values")
        self.x[2] = wrap_angle(float(self.x[2]))
        self.P = (
            np.diag([0.02**2, 0.02**2, math.radians(3.0) ** 2])
            if covariance is None
            else np.asarray(covariance, dtype=float).reshape(3, 3).copy()
        )

    def predict(self, body_delta) -> np.ndarray:
        """Integrate body [right, forward, yaw] and grow motion uncertainty."""
        dr, df, dyaw = np.asarray(body_delta, dtype=float)
        mid = self.x[2] + dyaw / 2.0
        c, s = math.cos(mid), math.sin(mid)
        world_delta = np.array([c * dr - s * df, s * dr + c * df, dyaw])

        F = np.eye(3)
        F[0, 2] = -s * dr - c * df
        F[1, 2] = c * dr - s * df
        distance = math.hypot(dr, df)
        position_std = 0.0005 + 0.04 * distance
        heading_std = math.radians(0.03) + 0.04 * abs(dyaw) + 0.01 * distance
        Q = np.diag([position_std**2, position_std**2, heading_std**2])

        self.x += world_delta
        self.x[2] = wrap_angle(float(self.x[2]))
        self.P = F @ self.P @ F.T + Q
        self.P = (self.P + self.P.T) / 2.0
        return self.x.copy()

    def update_waypoint(
        self,
        right_m: float,
        forward_m: float,
        heading_rad: float = 0.0,
        position_std_m: float = 0.05,
        heading_std_rad: float = math.radians(3.0),
    ) -> np.ndarray:
        """Fuse a surveyed floor mark. Heading is observed when parked facing the wall."""
        z = np.array([right_m, forward_m, heading_rad], dtype=float)
        residual = z - self.x
        residual[2] = wrap_angle(float(residual[2]))
        R = np.diag(
            [max(position_std_m, 0.01) ** 2] * 2
            + [max(heading_std_rad, math.radians(0.5)) ** 2]
        )
        S = self.P + R
        K = np.linalg.solve(S.T, self.P.T).T
        self.x += K @ residual
        self.x[2] = wrap_angle(float(self.x[2]))
        I_K = np.eye(3) - K
        self.P = I_K @ self.P @ I_K.T + K @ R @ K.T
        self.P = (self.P + self.P.T) / 2.0
        return self.x.copy()

    def reset(self, pose, position_std_m=0.01, heading_std_rad=math.radians(1.0)):
        self.x = np.asarray(pose, dtype=float).copy()
        self.x[2] = wrap_angle(float(self.x[2]))
        self.P = np.diag([position_std_m**2] * 2 + [heading_std_rad**2])

    def covariance_flat(self) -> list[float]:
        return self.P.reshape(-1).tolist()
