#!/usr/bin/env python3

import argparse
import json
import math
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class TrackingMeasurement(Node):
    # Set up arm state tracking and the action client.
    def __init__(self):
        super().__init__("measure_arm_tracking")
        self.latest = None
        self.samples = []
        self.create_subscription(JointState, "/joint_states", self._on_state, 100)
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

    # Store the latest arm joint positions and velocities.
    def _on_state(self, message):
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        if not all(name in positions for name in JOINTS):
            return
        self.latest = (
            [positions[name] for name in JOINTS],
            [velocities.get(name, math.nan) for name in JOINTS],
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
        nargs=6,
        type=float,
        default=[0.2, -1.3, 0.3, -1.4, 0.0, 0.0],
    )
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--settle", type=float, default=5.0)
    parser.add_argument("--final-window", type=float, default=1.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = TrackingMeasurement()
    try:
        if not node.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("arm_controller action server is unavailable")

        state_deadline = time.monotonic() + 10.0
        while node.latest is None and time.monotonic() < state_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.latest is None:
            raise RuntimeError("No complete /joint_states sample was received")

        initial = node.latest[0]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = list(args.target)
        whole_seconds = int(args.duration)
        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = int(
            (args.duration - whole_seconds) * 1_000_000_000
        )
        goal.trajectory.points = [point]

        send_future = node.client.send_goal_async(goal)
        if not wait_for_future(node, send_future, 10.0):
            raise RuntimeError("Timed out sending trajectory goal")
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Trajectory goal was rejected")
        result_future = goal_handle.get_result_async()

        start = time.monotonic()
        total_time = args.duration + args.settle
        while rclpy.ok() and time.monotonic() - start < total_time:
            rclpy.spin_once(node, timeout_sec=0.01)
            if node.latest is not None:
                node.samples.append(
                    (time.monotonic() - start, *node.latest)
                )

        if not node.samples:
            raise RuntimeError("No tracking samples were recorded")

        final_samples = [
            sample
            for sample in node.samples
            if sample[0] >= total_time - args.final_window
        ]
        if not final_samples:
            final_samples = node.samples[-1:]

        final_position = [
            sum(sample[1][index] for sample in final_samples) / len(final_samples)
            for index in range(len(JOINTS))
        ]
        final_error = [
            final_position[index] - args.target[index]
            for index in range(len(JOINTS))
        ]
        rms_error = [
            math.sqrt(
                sum(
                    (sample[1][index] - args.target[index]) ** 2
                    for sample in final_samples
                )
                / len(final_samples)
            )
            for index in range(len(JOINTS))
        ]
        peak_settled_velocity = [
            max(abs(sample[2][index]) for sample in final_samples)
            for index in range(len(JOINTS))
        ]

        result_status = None
        result_error_code = None
        if result_future.done():
            result = result_future.result()
            result_status = int(result.status)
            result_error_code = int(result.result.error_code)

        report = {
            "joints": JOINTS,
            "initial_position": initial,
            "target_position": args.target,
            "final_position_mean": final_position,
            "final_error": final_error,
            "rms_error_final_window": rms_error,
            "peak_velocity_final_window": peak_settled_velocity,
            "max_abs_final_error": max(abs(value) for value in final_error),
            "max_rms_error": max(rms_error),
            "max_settled_velocity": max(peak_settled_velocity),
            "action_status": result_status,
            "controller_error_code": result_error_code,
            "sample_count": len(node.samples),
        }
        print(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
