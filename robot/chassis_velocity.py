#!/usr/bin/env python3
"""Supervised, low-speed TA2 chassis command with fail-safe zero publishing."""

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String

from ta2_common import DIRECTION_COMMANDS, validate_duration, validate_velocity


class ChassisCommand(Node):
    def __init__(self) -> None:
        super().__init__("ta2_supervised_chassis_command")
        self.mode = None
        self.mode_at = 0.0
        self.wheel_velocity = None
        self.state_at = 0.0
        self.create_subscription(String, "/api/current_mode", self._on_mode, 10)
        self.create_subscription(JointState, "/chassis/joint_states", self._on_state, 10)
        self.publisher = None

    def _on_mode(self, message: String) -> None:
        try:
            self.mode = json.loads(message.data)
            self.mode_at = time.monotonic()
        except (TypeError, ValueError):
            pass

    def _on_state(self, message: JointState) -> None:
        self.wheel_velocity = list(message.velocity)
        self.state_at = time.monotonic()

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def require_ready(self) -> None:
        self.spin_for(3.0)
        if time.monotonic() - self.mode_at > 2.0:
            raise RuntimeError("current mode is stale")
        if self.mode.get("meta_mode") != 1 or not self.mode.get("enable_chassis"):
            raise RuntimeError("robot is not in API mode with chassis enabled")
        if time.monotonic() - self.state_at > 0.5 or len(self.wheel_velocity or []) != 3:
            raise RuntimeError("fresh chassis state was not received")
        if not all(math.isfinite(value) for value in self.wheel_velocity):
            raise RuntimeError("invalid chassis state")
        if self.count_publishers("/api/chassis/velocity") != 0:
            raise RuntimeError("another chassis command publisher is active")
        if self.count_subscribers("/api/chassis/velocity") < 1:
            raise RuntimeError("no chassis command subscriber is available")

    def run(self, velocity: tuple[float, float, float], duration: float) -> None:
        self.publisher = self.create_publisher(
            Float32MultiArray, "/api/chassis/velocity", 10
        )
        self.spin_for(0.5)
        deadline = time.monotonic() + duration
        command = Float32MultiArray(data=list(velocity))
        while time.monotonic() < deadline:
            if time.monotonic() - self.mode_at > 2.0 or time.monotonic() - self.state_at > 0.5:
                raise RuntimeError("robot state became stale")
            if self.count_publishers("/api/chassis/velocity") > 1:
                raise RuntimeError("competing chassis publisher detected")
            self.publisher.publish(command)
            self.spin_for(0.05)

    def stop(self) -> None:
        if self.publisher is None:
            return
        zero = Float32MultiArray(data=[0.0, 0.0, 0.0])
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.publisher.publish(zero)
            self.spin_for(0.05)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--direction", choices=sorted(DIRECTION_COMMANDS))
    group.add_argument("--velocity", nargs=3, type=float, metavar=("VX", "VY", "WZ"))
    parser.add_argument("--duration", required=True, type=float)
    parser.add_argument(
        "--execute", action="store_true", help="required to publish a non-zero command"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    raw_velocity = DIRECTION_COMMANDS[args.direction] if args.direction else args.velocity
    velocity = validate_velocity(*raw_velocity)
    duration = validate_duration(args.duration)
    if not args.execute:
        print(f"DRY RUN velocity={velocity}, duration={duration:.2f}s")
        return

    rclpy.init()
    node = ChassisCommand()
    try:
        node.require_ready()
        print(f"EXECUTING velocity={velocity}, duration={duration:.2f}s", flush=True)
        node.run(velocity, duration)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
