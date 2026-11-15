#!/usr/bin/env python3
"""Move both TA2 arms to the verified project start pose over five seconds."""

import json
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String


TARGETS = {
    "left_arm": [0.41, 1.16, -0.47, 0.90, 0.23, -0.15, 0.60],
    "right_arm": [-0.50, -0.92, 0.52, -1.28, 0.32, 0.55, -0.52],
}


class ArmInterpolator(Node):
    def __init__(self, arm: str, enable: bool = False) -> None:
        super().__init__(f"{arm}_safe_interpolator")
        self.arm = arm
        self.target = np.asarray(TARGETS[arm], dtype=float)
        self.current = None
        self.start = None
        self.converged_at = None
        self.done = False
        self.armed = False
        self.mode = None
        self.mode_at = 0.0
        self.joint_names = []
        self.create_subscription(JointState, f"/{arm}/joint_states", self._state, 10)
        self.create_subscription(String, "/api/current_mode", self._mode, 10)
        self.publisher = self.create_publisher(JointState, f"/api/{arm}/joint_cmd", 10)
        if enable:
            self.enable_publisher = self.create_publisher(Float32, "/api/fsm/enable", 10)
            self.create_timer(0.02, self._enable)
        self.create_timer(0.01, self._tick)

    def _mode(self, message: String) -> None:
        try:
            self.mode = json.loads(message.data)
            self.mode_at = time.monotonic()
        except (TypeError, ValueError):
            pass

    def _enable(self) -> None:
        if self.armed:
            self.enable_publisher.publish(Float32(data=1.0))

    def _state(self, message: JointState) -> None:
        if len(message.position) < 7 or len(message.name) < 7:
            return
        self.joint_names = list(message.name[:7])
        self.current = np.asarray(message.position[:7], dtype=float)
        if self.start is None:
            self.start = self.get_clock().now()
            self.initial = self.current.copy()

    def _tick(self) -> None:
        if not self.armed or self.current is None or self.start is None:
            return
        elapsed = (self.get_clock().now() - self.start).nanoseconds / 1e9
        alpha = min(1.0, elapsed / 5.0)
        desired = self.initial * (1.0 - alpha) + self.target * alpha
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self.joint_names
        message.position = desired.tolist()
        message.velocity = [0.0] * 7
        message.effort = [0.0] * 7
        self.publisher.publish(message)
        if elapsed >= 5.0 and np.max(np.abs(self.current - self.target)) < 0.05:
            self.converged_at = self.converged_at or time.monotonic()
            self.done = time.monotonic() - self.converged_at >= 0.5
        else:
            self.converged_at = None


def main() -> None:
    rclpy.init()
    left = ArmInterpolator("left_arm", enable=True)
    right = ArmInterpolator("right_arm")
    executor = MultiThreadedExecutor()
    executor.add_node(left)
    executor.add_node(right)
    deadline = time.monotonic() + 20.0
    try:
        ready_deadline = time.monotonic() + 4.0
        while time.monotonic() < ready_deadline and (
            left.current is None or right.current is None or left.mode is None
        ):
            executor.spin_once(timeout_sec=0.1)
        if left.current is None or right.current is None:
            raise RuntimeError("fresh joint states were not received")
        if left.count_publishers("/api/left_arm/joint_cmd") != 1:
            raise RuntimeError("another left-arm publisher is active")
        if right.count_publishers("/api/right_arm/joint_cmd") != 1:
            raise RuntimeError("another right-arm publisher is active")
        if time.monotonic() - left.mode_at > 2.0:
            raise RuntimeError("current mode is stale")
        mode = left.mode
        if (
            mode.get("meta_mode") != 1
            or mode.get("left_arm_control_mode") != 1
            or mode.get("right_arm_control_mode") != 1
            or not mode.get("enable_left_arm")
            or not mode.get("enable_right_arm")
        ):
            raise RuntimeError("robot is not in API mode with both arms in joint mode")
        left.armed = True
        right.armed = True
        while time.monotonic() < deadline and not (left.done and right.done):
            executor.spin_once(timeout_sec=0.1)
        if not (left.done and right.done):
            raise RuntimeError("arms did not converge within 20 seconds")
        print("Both arms converged to the project start pose.")
    finally:
        stop = left.create_publisher(Float32, "/api/fsm/enable", 10)
        for _ in range(20):
            stop.publish(Float32(data=0.0))
            executor.spin_once(timeout_sec=0.02)
        left.destroy_node()
        right.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
