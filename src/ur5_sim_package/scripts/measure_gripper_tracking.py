#!/usr/bin/env python3

import argparse
import json
import math
import time

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


COMMAND_JOINT = "left_finger_joint"
STATE_JOINTS = ["left_finger_joint"]


class GripperMeasurement(Node):
    # Set up gripper state tracking and the action client.
    def __init__(self):
        super().__init__("measure_gripper_tracking")
        self.latest = None
        self.samples = []
        self.create_subscription(JointState, "/joint_states", self._on_state, 100)
        self.client = ActionClient(
            self,
            GripperCommand,
            "/gripper_controller/gripper_cmd",
        )

    # Store the latest gripper position and velocity.
    def _on_state(self, message):
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        if not all(name in positions for name in STATE_JOINTS):
            return
        self.latest = (
            [positions[name] for name in STATE_JOINTS],
            [velocities.get(name, math.nan) for name in STATE_JOINTS],
        )


# Wait for a ROS action result or timeout.
def wait_for_future(node, future, timeout):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    return future.done()


# Send the test goal and print the tracking results.
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--target",
        type=float,
        default=0.035,
        help="Symmetric finger position in metres (closed=0.0, open=0.035).",
    )
    parser.add_argument("--max-effort", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=6.0)
    parser.add_argument("--settle", type=float, default=1.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = GripperMeasurement()
    try:
        if not node.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("gripper_controller action server is unavailable")

        deadline = time.monotonic() + 10.0
        while node.latest is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.latest is None:
            raise RuntimeError("No complete gripper /joint_states sample was received")

        initial = node.latest[0]
        goal = GripperCommand.Goal()
        goal.command.position = args.target
        goal.command.max_effort = args.max_effort

        send_future = node.client.send_goal_async(goal)
        if not wait_for_future(node, send_future, 10.0):
            raise RuntimeError("Timed out sending gripper goal")
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Gripper goal was rejected")
        result_future = goal_handle.get_result_async()

        start = time.monotonic()
        while (
            rclpy.ok()
            and not result_future.done()
            and time.monotonic() - start < args.timeout
        ):
            rclpy.spin_once(node, timeout_sec=0.01)
            if node.latest is not None:
                node.samples.append((time.monotonic() - start, *node.latest))

        settle_deadline = time.monotonic() + args.settle
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            if node.latest is not None:
                node.samples.append((time.monotonic() - start, *node.latest))

        final_position = node.samples[-1][1]
        final_velocity = node.samples[-1][2]
        errors = [value - args.target for value in final_position]

        result_status = None
        reached_goal = None
        stalled = None
        applied_effort = None
        if result_future.done():
            result = result_future.result()
            result_status = int(result.status)
            reached_goal = bool(result.result.reached_goal)
            stalled = bool(result.result.stalled)
            applied_effort = float(result.result.effort)

        print(
            json.dumps(
                {
                    "state_joints": STATE_JOINTS,
                    "command_joint": COMMAND_JOINT,
                    "initial_position": initial,
                    "target_position": [args.target] * len(STATE_JOINTS),
                    "max_effort": args.max_effort,
                    "final_position": final_position,
                    "final_error": errors,
                    "final_velocity": final_velocity,
                    "max_abs_final_error": max(abs(value) for value in errors),
                    "action_status": result_status,
                    "reached_goal": reached_goal,
                    "stalled": stalled,
                    "applied_effort": applied_effort,
                    "sample_count": len(node.samples),
                },
                indent=2,
            )
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
