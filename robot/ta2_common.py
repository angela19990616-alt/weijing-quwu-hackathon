"""Pure helpers shared by the TA2 ROS 2 hardware scripts."""

import math


MAX_LINEAR_SPEED = 0.05
MAX_ANGULAR_SPEED = 0.20

# Measured on 2026-11-15: [vx=+0.05, vy=0, wz=0] for two seconds moved the
# robot roughly 3-5 cm toward its physical right. The remaining axes are not
# calibrated yet, so only the measured right direction has a named shortcut.
DIRECTION_COMMANDS = {
    "right": (MAX_LINEAR_SPEED, 0.0, 0.0),
}


def validate_velocity(vx: float, vy: float, wz: float) -> tuple[float, float, float]:
    values = (float(vx), float(vy), float(wz))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("velocity values must be finite")
    if math.hypot(values[0], values[1]) > MAX_LINEAR_SPEED + 1e-12:
        raise ValueError(f"linear speed exceeds {MAX_LINEAR_SPEED:.2f} m/s")
    if abs(values[2]) > MAX_ANGULAR_SPEED:
        raise ValueError(f"angular speed exceeds {MAX_ANGULAR_SPEED:.2f} rad/s")
    return values


def validate_duration(duration: float) -> float:
    duration = float(duration)
    if not math.isfinite(duration) or not 0 < duration <= 5.0:
        raise ValueError("duration must be in (0, 5] seconds")
    return duration
