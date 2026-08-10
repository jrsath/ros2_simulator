#!/usr/bin/env python3
import math
import threading
import time
from copy import deepcopy

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose, PoseStamped
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

BLUE = "\033[1;94m"
RESET = "\033[0m"

class PickAndPlaceNode(Node):
    # Set up the node, parameters, actions and services.
    def __init__(self):
        super().__init__("pick_and_place_node")

        self._declare_parameters()
        self.world_frame = self._string("world_frame")
        self.arm_group = self._string("arm_group")
        self.tcp_link = self._string("tcp_link")
        self.car_id = self._string("car_id")

        self.car_x = self._double("car_x")
        self.car_y = self._double("car_y")
        self.car_z = self._double("car_z")
        self.car_dimensions = [
            self._double("car_length"),
            self._double("car_width"),
            self._double("car_height"),
        ]
        self.gripper_yaw = self._double("gripper_yaw")
        self.place_roll = self._double("place_roll")
        self.place_yaw = self._double("place_yaw")
        self.tcp_x_offset = self._double("tcp_x_offset")
        self.tcp_y_offset = self._double("tcp_y_offset")
        self.grasp_z = self._double("grasp_tcp_z")
        self.pre_grasp_height = self._double("pre_grasp_height")
        self.lift_height = self._double("lift_height")
        self.pedestal_x = self._double("pedestal_x")
        self.pedestal_y = self._double("pedestal_y")
        self.pedestal_top = self._double("pedestal_top")
        self.place_x_offset = self._double("place_x_offset")
        self.place_y_offset = self._double("place_y_offset")
        self.place_clearance = self._double("place_clearance")
        self.vertical_step = self._double("vertical_step")
        self.lift_velocity_scaling = self._double("lift_velocity_scaling")
        self.lift_acceleration_scaling = self._double(
            "lift_acceleration_scaling"
        )
        self.plan_retries = self._integer("plan_retries")
        self.ik_candidates = self._integer("ik_candidates")
        self.ik_timeout = self._double("ik_timeout")
        self.position_tolerance = self._double("position_tolerance")
        self.orientation_tolerance = self._double("orientation_tolerance")
        self.future_timeout = self._double("future_timeout")
        self.startup_timeout = self._double("startup_timeout")
        self.open_position = self._double("gripper_open_position")
        self.closed_position = self._double("gripper_closed_position")
        self.gripper_effort = self._double("gripper_max_effort")
        self.gripper_grasp_effort = self._double("gripper_grasp_effort")
        self.gripper_release_effort = self._double("gripper_release_effort")
        self.gripper_settle_position = self._double(
            "gripper_settle_position"
        )
        self.gripper_feedback_tolerance = self._double(
            "gripper_feedback_tolerance"
        )
        self.gripper_contact_min_position = self._double(
            "gripper_contact_min_position"
        )
        self.gripper_contact_max_position = self._double(
            "gripper_contact_max_position"
        )
        self.pedestal_contact_height = self._double(
            "pedestal_contact_height"
        )
        self.pedestal_contact_tolerance = self._double(
            "pedestal_contact_tolerance"
        )
        self.drive_clearance_distance = self._double(
            "drive_clearance_distance"
        )
        self.place_wrist_step = self._double("place_wrist_step")
        self.place_target_tilt = self._double("place_target_tilt")
        self.place_wrist_max_steps = self._integer(
            "place_wrist_max_steps"
        )
        self.pregrasp_joints = self._double_array("pregrasp_joints")
        self.pedestal_joints = self._double_array("pedestal_joints")
        self.drive_clearance_joints = self._double_array(
            "drive_clearance_joints"
        )
        self.release_retreat_joints = self._double_array(
            "release_retreat_joints"
        )

        self._scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self._scene_query_client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self._controller_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._joint_state_lock = threading.Lock()
        self._joint_positions = {}
        self._joint_state_event = threading.Event()
        self._joint_state_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )
        self._car_pose_lock = threading.Lock()
        self._car_height = None
        self._car_pose = None
        self._car_pose_event = threading.Event()
        self._car_pose_subscription = self.create_subscription(
            TFMessage,
            "/world/empty/dynamic_pose/info",
            self._car_pose_callback,
            10,
        )
        self._gripper_client = ActionClient(
            self,
            GripperCommand,
            self._string("gripper_action"),
        )
        self._trigger_car_publisher = self.create_publisher(
            Bool,
            "/trigger_car",
            10,
        )

        self._car_removed = False
        self._car_collision_attached = False
        self._moveit = None
        self._arm = None
        self._plan_parameters = None

    # Declare all task settings.
    def _declare_parameters(self):
        defaults = {
            "world_frame": "world",
            "arm_group": "ur_manipulator",
            "tcp_link": "gripper_tcp",
            "gripper_action": "/gripper_controller/gripper_cmd",
            "car_id": "two_wheel_car",
            # Car outer bounds.
            "car_x": 0.224,
            "car_y": 0.0,
            "car_z": 0.02,
            "car_length": 0.10,
            "car_width": 0.07,
            "car_height": 0.04,
            # Grasp alignment.
            "gripper_yaw": math.pi,
            "place_roll": math.radians(70.0),
            "place_yaw": math.radians(-90.0),
            "tcp_x_offset": 0.050,
            "tcp_y_offset": 0.0,
            "grasp_tcp_z": 0.025,
            "pre_grasp_height": 0.075,
            "lift_height": 0.30,
            "pedestal_x": 0.50,
            "pedestal_y": 0.0,
            "pedestal_top": 0.80,
            # Placement offsets.
            "place_x_offset": 0.0,
            "place_y_offset": 0.0,
            "place_clearance": 0.025,
            "vertical_step": 0.10,
            "lift_velocity_scaling": 0.32,
            "lift_acceleration_scaling": 0.24,
            "plan_retries": 3,
            "ik_candidates": 20,
            "ik_timeout": 0.05,
            "position_tolerance": 0.03,
            "orientation_tolerance": 0.10,
            "future_timeout": 30.0,
            "startup_timeout": 90.0,
            "gripper_open_position": 0.035,
            "gripper_closed_position": 0.0,
            "gripper_max_effort": 20.0,
            "gripper_grasp_effort": 50.0,
            "gripper_release_effort": 50.0,
            "gripper_settle_position": 0.030,
            "gripper_feedback_tolerance": 0.0005,
            # Expected contact below the fully open limit.
            "gripper_contact_min_position": 0.023,
            "gripper_contact_max_position": 0.033,
            "pedestal_contact_height": 0.850,
            "pedestal_contact_tolerance": 0.005,
            "drive_clearance_distance": 0.010,
            "place_wrist_step": math.radians(10.0),
            "place_target_tilt": math.radians(73.0),
            "place_wrist_max_steps": 10,
            # Collision-free branches for the configured robot base.
            "pregrasp_joints": [
                0.228394,
                -0.764886,
                -2.368938,
                -1.578557,
                1.570795,
                -2.913188,
            ],
            "pedestal_joints": [
                -0.089285,
                -1.591031,
                0.011439,
                -2.452446,
                2.824589,
                2.123493,
            ],
            "release_retreat_joints": [
                -0.078571,
                -1.544107,
                -0.109934,
                -2.302152,
                3.119620,
                1.868674,
            ],
            "drive_clearance_joints": [
                -0.082143,
                -1.559748,
                -0.069477,
                -2.352250,
                3.021277,
                1.953613,
            ],
            "planning_time": 10.0,
            "planning_attempts": 5,
            "velocity_scaling": 0.20,
            "acceleration_scaling": 0.16,
            "planner_id": "RRTConnect",
            "planning_pipeline": "ompl",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    # Read a string parameter.
    def _string(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    # Read a decimal parameter.
    def _double(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    # Read an integer parameter.
    def _integer(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value

    # Read a list of decimal values.
    def _double_array(self, name):
        return list(
            self.get_parameter(name).get_parameter_value().double_array_value
        )

    # Print a blue task state message.
    def _log_state(self, message):
        self.get_logger().info(f"{BLUE}{message}{RESET}")

    # Create the MoveIt planning components.
    def initialise_moveit(self):
        self.get_logger().info("Initialising the MoveIt 2 Python API")
        self._moveit = MoveItPy(
            node_name="pick_and_place_moveit_py",
            provide_planning_service=False,
        )
        self._arm = self._moveit.get_planning_component(self.arm_group)
        self._plan_parameters = PlanRequestParameters(self._moveit)
        self._plan_parameters.planner_id = self._string("planner_id")
        self._plan_parameters.planning_pipeline = self._string(
            "planning_pipeline"
        )
        self._plan_parameters.planning_attempts = self._integer(
            "planning_attempts"
        )
        self._plan_parameters.planning_time = self._double("planning_time")
        self._plan_parameters.max_velocity_scaling_factor = self._double(
            "velocity_scaling"
        )
        self._plan_parameters.max_acceleration_scaling_factor = self._double(
            "acceleration_scaling"
        )

    # Store the latest joint positions.
    def _joint_state_callback(self, message):
        with self._joint_state_lock:
            self._joint_positions.update(zip(message.name, message.position))
        required = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
            "left_finger_joint",
        }
        if required.issubset(message.name):
            self._joint_state_event.set()

    # Return the latest position of one joint.
    def _joint_position(self, name):
        with self._joint_state_lock:
            return self._joint_positions.get(name)

    # Store the physical car height reported by Gazebo.
    def _car_pose_callback(self, message):
        for transform in message.transforms:
            if transform.child_frame_id != self.car_id:
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            with self._car_pose_lock:
                self._car_height = translation.z
                self._car_pose = (
                    translation.x,
                    translation.y,
                    translation.z,
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                )
            self._car_pose_event.set()
            return

    # Return the car tilt from wheels-down.
    def _car_tilt(self):
        with self._car_pose_lock:
            pose = self._car_pose
        if pose is None:
            return None
        upright = 1.0 - 2.0 * (pose[3] * pose[3] + pose[4] * pose[4])
        return math.acos(max(-1.0, min(1.0, upright)))

    # Return the vertical gap below the car collision box.
    def _car_pedestal_gap(self):
        with self._car_pose_lock:
            car_pose = self._car_pose
        if car_pose is None:
            return None

        model_pose = Pose()
        model_pose.position.x = car_pose[0]
        model_pose.position.y = car_pose[1]
        model_pose.position.z = car_pose[2]
        model_pose.orientation.x = car_pose[3]
        model_pose.orientation.y = car_pose[4]
        model_pose.orientation.z = car_pose[5]
        model_pose.orientation.w = car_pose[6]
        center, half = self._box_aabb(
            model_pose,
            (0.024, 0.0, -0.030),
            (0.10, 0.07, 0.04),
        )
        return center[2] - half[2] - self.pedestal_top

    # Wait until the car is upright and resting on the pedestal.
    def _wait_for_pedestal_contact(self, timeout=3.0):
        deadline = time.monotonic() + timeout
        stable_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            with self._car_pose_lock:
                pose = self._car_pose
            if pose is None:
                time.sleep(0.05)
                continue

            x, y, z, qx, qy, _, _ = pose
            upright = 1.0 - 2.0 * (qx * qx + qy * qy)
            on_pedestal = (
                abs(z - self.pedestal_contact_height)
                <= self.pedestal_contact_tolerance
                # Keep the requested 5 mm edge margin with 1 mm tolerance.
                and abs(x - self.pedestal_x) <= 0.046
                and abs(y - self.pedestal_y) <= 0.060
                and upright >= math.cos(math.radians(15.0))
            )
            if on_pedestal:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= 0.30:
                    self.get_logger().info(
                        "Car contact verified: "
                        f"position=({x:.3f}, {y:.3f}, {z:.3f})"
                    )
                    return True
            else:
                stable_since = None
            time.sleep(0.05)

        with self._car_pose_lock:
            final_pose = self._car_pose
        if final_pose is None:
            self.get_logger().warning("No Gazebo car pose was available")
        else:
            x, y, z, qx, qy, _, _ = final_pose
            tilt = math.degrees(
                math.acos(
                    max(-1.0, min(1.0, 1.0 - 2.0 * (qx * qx + qy * qy)))
                )
            )
            self.get_logger().warning(
                "Car did not satisfy the placement window: "
                f"position=({x:.3f}, {y:.3f}, {z:.3f}), "
                f"tilt={tilt:.1f} deg"
            )
        return False

    # Return a world AABB for a box fixed to a link pose.
    def _box_aabb(self, link_pose, local_center, dimensions):
        quaternion = (
            link_pose.orientation.x,
            link_pose.orientation.y,
            link_pose.orientation.z,
            link_pose.orientation.w,
        )
        offset = self._rotate_by_quaternion(quaternion, local_center)
        center = (
            link_pose.position.x + offset[0],
            link_pose.position.y + offset[1],
            link_pose.position.z + offset[2],
        )

        x, y, z, w = quaternion
        rotation = (
            (
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ),
            (
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ),
            (
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ),
        )
        local_half = tuple(value * 0.5 for value in dimensions)
        half = tuple(
            sum(
                abs(rotation[row][column]) * local_half[column]
                for column in range(3)
            )
            for row in range(3)
        )
        return center, half

    # Measure the gap between the finger collision boxes and the car box.
    def _finger_car_clearance(self):
        with self._car_pose_lock:
            car_pose = self._car_pose
        if car_pose is None:
            return None

        car_model = Pose()
        car_model.position.x = car_pose[0]
        car_model.position.y = car_pose[1]
        car_model.position.z = car_pose[2]
        car_model.orientation.x = car_pose[3]
        car_model.orientation.y = car_pose[4]
        car_model.orientation.z = car_pose[5]
        car_model.orientation.w = car_pose[6]
        car_center, car_half = self._box_aabb(
            car_model,
            (0.024, 0.0, -0.030),
            (0.10, 0.07, 0.04),
        )

        monitor = self._moveit.get_planning_scene_monitor()
        with monitor.read_only() as scene:
            link_poses = {
                name: deepcopy(scene.current_state.get_pose(name))
                for name in ("left_finger_link", "right_finger_link")
            }

        finger_boxes = (
            ((0.0157, -0.065, 0.0075), (0.0306, 0.080, 0.015)),
            ((0.0414, -0.01275, 0.0785), (0.025, 0.0255, 0.117)),
        )
        minimum = math.inf
        for link_pose in link_poses.values():
            for local_center, dimensions in finger_boxes:
                finger_center, finger_half = self._box_aabb(
                    link_pose,
                    local_center,
                    dimensions,
                )
                separation = tuple(
                    max(
                        abs(finger_center[axis] - car_center[axis])
                        - finger_half[axis]
                        - car_half[axis],
                        0.0,
                    )
                    for axis in range(3)
                )
                distance = math.sqrt(sum(value * value for value in separation))
                minimum = min(minimum, distance)
        return minimum

    # Check that Gazebo shows the car above the required height.
    def _car_is_above(self, minimum_height, description):
        if not self._car_pose_event.wait(timeout=2.0):
            self.get_logger().error(f"{description}: no Gazebo car pose received")
            return False
        with self._car_pose_lock:
            height = self._car_height
        if height is not None and height >= minimum_height:
            self.get_logger().info(
                f"{description}: physical car height verified at {height:.3f} m"
            )
            return True
        self.get_logger().error(
            f"{description}: physical car was dropped at height {height}"
        )
        return False

    # Check that the car is still between the fingers.
    def _grip_is_held(self, description):
        time.sleep(0.25)
        position = self._joint_position("left_finger_joint")
        if (
            position is not None
            and self.gripper_contact_min_position
            <= position
            <= self.gripper_contact_max_position
        ):
            self.get_logger().info(
                f"{description}: grip verified at {position:.4f} m"
            )
            return True
        self.get_logger().error(
            f"{description}: grip was lost; finger position={position}"
        )
        return False

    # Wait for MoveIt, controllers and joint data.
    def _wait_until_ready(self):
        required = {
            "arm_controller",
            "gripper_controller",
            "joint_state_broadcaster",
        }
        deadline = time.monotonic() + self.startup_timeout
        self.get_logger().info(
            "Waiting for active arm/gripper controllers and joint feedback"
        )

        while rclpy.ok() and time.monotonic() < deadline:
            if not self._controller_client.wait_for_service(timeout_sec=1.0):
                continue
            future = self._controller_client.call_async(ListControllers.Request())
            response_deadline = min(deadline, time.monotonic() + 5.0)
            while (
                rclpy.ok()
                and not future.done()
                and time.monotonic() < response_deadline
            ):
                time.sleep(0.05)
            if not future.done():
                self.get_logger().warning(
                    "Controller-manager query timed out; retrying"
                )
                continue
            try:
                response = future.result()
            except Exception as error:
                self.get_logger().warning(
                    f"Controller-manager query failed: {error}; retrying"
                )
                continue

            active = {
                controller.name
                for controller in response.controller
                if controller.state == "active"
            }
            missing = required - active
            if not missing and self._joint_state_event.wait(timeout=2.0):
                if self._gripper_client.wait_for_server(timeout_sec=1.0):
                    self.get_logger().info(
                        "Controllers are active and joint feedback is available"
                    )
                    return True
            if missing:
                self.get_logger().info(
                    "Still waiting for controllers: " + ", ".join(sorted(missing))
                )
            else:
                self.get_logger().info("Still waiting for /joint_states feedback")
            time.sleep(0.5)

        self.get_logger().error(
            "Robot did not become ready before the startup timeout"
        )
        return False

    # Convert roll, pitch and yaw to a rotation matrix.
    @staticmethod
    def _rotation_from_rpy(roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )

    # Rotate a vector with a rotation matrix.
    @staticmethod
    def _rotate_vector(rotation, vector):
        return tuple(
            sum(rotation[row][column] * vector[column] for column in range(3))
            for row in range(3)
        )

    # Convert roll, pitch and yaw to a quaternion.
    @staticmethod
    def _quaternion_from_rpy(roll, pitch, yaw):
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    # Multiply two quaternions.
    @staticmethod
    def _multiply_quaternions(first, second):
        x1, y1, z1, w1 = first
        x2, y2, z2, w2 = second
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    # Rotate a vector with a quaternion.
    @staticmethod
    def _rotate_by_quaternion(quaternion, vector):
        x, y, z, w = quaternion
        rotation = (
            (
                1 - 2 * (y * y + z * z),
                2 * (x * y - z * w),
                2 * (x * z + y * w),
            ),
            (
                2 * (x * y + z * w),
                1 - 2 * (x * x + z * z),
                2 * (y * z - x * w),
            ),
            (
                2 * (x * z - y * w),
                2 * (y * z + x * w),
                1 - 2 * (x * x + y * y),
            ),
        )
        return PickAndPlaceNode._rotate_vector(rotation, vector)

    # Build a stamped pose from position and RPY values.
    def _pose_from_rpy(self, frame_id, x, y, z, roll, pitch, yaw):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        quaternion = self._quaternion_from_rpy(roll, pitch, yaw)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose

    # Build a downward-facing gripper pose.
    def _downward_pose(self, frame_id, x, y, z):
        return self._pose_from_rpy(
            frame_id,
            x,
            y,
            z,
            math.pi,
            0.0,
            self.gripper_yaw,
        )

    # Find the car position relative to the gripper tip.
    def _car_translation_in_tcp(self):
        world_dx = -self.tcp_x_offset
        world_dy = -self.tcp_y_offset
        yaw_cos = math.cos(self.gripper_yaw)
        yaw_sin = math.sin(self.gripper_yaw)
        return (
            yaw_cos * world_dx + yaw_sin * world_dy,
            yaw_sin * world_dx - yaw_cos * world_dy,
            self.grasp_z - self.car_z,
        )

    # Build the centred sideways placement pose.
    def _placement_pose(self):
        rotation = self._rotation_from_rpy(
            math.pi + self.place_roll,
            0.0,
            self.place_yaw,
        )
        roll_sin = abs(math.sin(self.place_roll))
        roll_cos = abs(math.cos(self.place_roll))
        car_lower_extent = 0.035 * roll_sin + 0.020 * roll_cos
        place_car_z = (
            self.pedestal_top + car_lower_extent + self.place_clearance
        )
        car_offset = self._rotate_vector(
            rotation,
            self._car_translation_in_tcp(),
        )
        place_pose = self._pose_from_rpy(
            self.world_frame,
            self.pedestal_x + self.place_x_offset - car_offset[0],
            self.pedestal_y + self.place_y_offset - car_offset[1],
            place_car_z - car_offset[2],
            math.pi + self.place_roll,
            0.0,
            self.place_yaw,
        )
        return place_pose

    # Wait for a service or action response.
    def _wait_for_future(self, future, description):
        deadline = time.monotonic() + self.future_timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                self.get_logger().warning(f"Timed out waiting for {description}")
                return None
            time.sleep(0.05)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception as error:  # ROS futures propagate service/action errors.
            self.get_logger().warning(f"{description} failed: {error}")
            return None

    # Plan and execute a target end-effector pose.
    def _plan_and_execute_pose(
        self,
        pose,
        description,
        ik_seed_name=None,
        position_tolerance=None,
        orientation_tolerance=None,
    ):
        accepted_position_error = (
            self.position_tolerance
            if position_tolerance is None
            else position_tolerance
        )
        accepted_orientation_error = (
            self.orientation_tolerance
            if orientation_tolerance is None
            else orientation_tolerance
        )
        for attempt in range(1, self.plan_retries + 1):
            self._arm.set_start_state_to_current_state()
            # Generate an IK goal from the current state.
            monitor = self._moveit.get_planning_scene_monitor()
            goal_state = None
            rejected_collisions = 0
            with monitor.read_only() as scene:
                current_seed = deepcopy(scene.current_state)
                arm_joint_group = current_seed.robot_model.get_joint_model_group(
                    self.arm_group
                )
                for candidate_index in range(self.ik_candidates):
                    candidate = deepcopy(current_seed)
                    if candidate_index == 0 and attempt == 1:
                        if attempt == 1 and ik_seed_name:
                            candidate.set_to_default_values(
                                self.arm_group,
                                ik_seed_name,
                            )
                    else:
                        candidate.set_to_random_positions(arm_joint_group)
                    candidate.update()
                    ik_found = candidate.set_from_ik(
                        self.arm_group,
                        pose.pose,
                        self.tcp_link,
                        timeout=self.ik_timeout,
                    )
                    if not ik_found:
                        continue
                    candidate.update()
                    if not scene.is_state_valid(
                        candidate,
                        self.arm_group,
                        False,
                    ):
                        rejected_collisions += 1
                        continue
                    goal_state = candidate
                    break

            if goal_state is None:
                self.get_logger().warning(
                    f"{description}: no collision-free IK found "
                    f"({rejected_collisions} colliding candidates rejected) "
                    f"(attempt {attempt}/{self.plan_retries}); retrying"
                )
                time.sleep(0.1)
                continue
            goal_state.update()
            goal_set = self._arm.set_goal_state(robot_state=goal_state)
            if not goal_set:
                self.get_logger().warning(
                    f"{description}: pose goal was rejected "
                    f"(attempt {attempt}/{self.plan_retries}); retrying"
                )
                continue

            plan = self._arm.plan(self._plan_parameters)
            if not plan:
                code = getattr(plan.error_code, "val", "unknown")
                self.get_logger().warning(
                    f"{description}: no trajectory found, MoveIt code {code} "
                    f"(attempt {attempt}/{self.plan_retries}); retrying"
                )
                continue

            self.get_logger().info(
                f"{description}: trajectory found; executing attempt {attempt}"
            )
            execution = self._moveit.execute(
                self.arm_group,
                plan.trajectory,
                blocking=True,
            )
            if execution:
                time.sleep(0.5)
                position_error, orientation_error = self._pose_error(pose)
                if (
                    position_error <= accepted_position_error
                    and orientation_error <= accepted_orientation_error
                ):
                    return True
                self.get_logger().warning(
                    f"{description}: controller reported success but TCP "
                    f"error is {position_error * 1000.0:.1f} mm / "
                    f"{math.degrees(orientation_error):.1f} deg "
                    f"(attempt {attempt}/{self.plan_retries}); correcting"
                )
                continue

            status = getattr(execution, "status", "unknown")
            self.get_logger().warning(
                f"{description}: trajectory execution failed ({status}) "
                f"on attempt {attempt}/{self.plan_retries}; replanning"
            )

        self.get_logger().error(
            f"{description}: failed after {self.plan_retries} attempts"
        )
        return False

    # Plan a faster move away from contact areas.
    def _plan_and_execute_fast_pose(self, pose, description, **kwargs):
        normal_velocity = self._plan_parameters.max_velocity_scaling_factor
        normal_acceleration = self._plan_parameters.max_acceleration_scaling_factor
        self._plan_parameters.max_velocity_scaling_factor = (
            self.lift_velocity_scaling
        )
        self._plan_parameters.max_acceleration_scaling_factor = (
            self.lift_acceleration_scaling
        )
        try:
            return self._plan_and_execute_pose(
                pose,
                description,
                **kwargs,
            )
        finally:
            self._plan_parameters.max_velocity_scaling_factor = normal_velocity
            self._plan_parameters.max_acceleration_scaling_factor = normal_acceleration

    # Calculate the error from a target pose.
    def _pose_error(self, target):
        actual = self._current_tcp_pose().pose

        dx = actual.position.x - target.pose.position.x
        dy = actual.position.y - target.pose.position.y
        dz = actual.position.z - target.pose.position.z
        position_error = math.sqrt(dx * dx + dy * dy + dz * dz)

        dot = abs(
            actual.orientation.x * target.pose.orientation.x
            + actual.orientation.y * target.pose.orientation.y
            + actual.orientation.z * target.pose.orientation.z
            + actual.orientation.w * target.pose.orientation.w
        )
        dot = min(1.0, max(-1.0, dot))
        orientation_error = 2.0 * math.acos(dot)
        return position_error, orientation_error

    # Read the measured gripper pose.
    def _current_tcp_pose(self):
        monitor = self._moveit.get_planning_scene_monitor()
        with monitor.read_only() as scene:
            actual = scene.current_state.get_pose(self.tcp_link)
        measured = PoseStamped()
        measured.header.frame_id = self.world_frame
        measured.pose = deepcopy(actual)
        return measured

    # Centre the gripper above the car before descending.
    def _centre_over_car(self, target):
        centring_attempts = 10
        for attempt in range(1, centring_attempts + 1):
            actual = self._current_tcp_pose()
            dx = target.pose.position.x - actual.pose.position.x
            dy = target.pose.position.y - actual.pose.position.y
            horizontal_error = math.hypot(dx, dy)
            self.get_logger().info(
                "Pre-grasp horizontal error: "
                f"{horizontal_error * 1000.0:.1f} mm"
            )
            if horizontal_error <= 0.006:
                return True

            correction = deepcopy(target)
            correction.pose.position.z = max(
                target.pose.position.z,
                actual.pose.position.z,
            )
            if not self._plan_and_execute_pose(
                correction,
                f"State 1 Pre-Grasp Centring {attempt}",
                position_tolerance=0.04,
                orientation_tolerance=math.radians(8.0),
            ):
                return False

        actual = self._current_tcp_pose()
        final_error = math.hypot(
            target.pose.position.x - actual.pose.position.x,
            target.pose.position.y - actual.pose.position.y,
        )
        if final_error <= 0.006:
            self.get_logger().info(
                f"Pre-grasp horizontal error: {final_error * 1000.0:.1f} mm"
            )
            return True
        self.get_logger().error("Gripper is not centred above the car")
        return False

    # Plan and execute a named robot pose.
    def _plan_and_execute_named(self, name, description):
        for attempt in range(1, self.plan_retries + 1):
            self._arm.set_start_state_to_current_state()
            goal_set = self._arm.set_goal_state(configuration_name=name)
            if not goal_set:
                self.get_logger().warning(
                    f"{description}: named state '{name}' was rejected; retrying"
                )
                continue
            plan = self._arm.plan(self._plan_parameters)
            if not plan:
                self.get_logger().warning(
                    f"{description}: no trajectory found "
                    f"(attempt {attempt}/{self.plan_retries}); retrying"
                )
                continue
            execution = self._moveit.execute(
                self.arm_group,
                plan.trajectory,
                blocking=True,
            )
            if execution:
                return True
            status = getattr(execution, "status", "unknown")
            self.get_logger().warning(
                f"{description}: execution failed ({status}); replanning"
            )
        self.get_logger().error(
            f"{description}: failed after {self.plan_retries} attempts"
        )
        return False

    # Plan and execute an arm joint target.
    def _plan_and_execute_joint_positions(
        self,
        positions,
        description,
        fast=False,
    ):
        if len(positions) != 6:
            self.get_logger().warning(
                f"{description}: expected six joint positions"
            )
            return False

        normal_velocity = self._plan_parameters.max_velocity_scaling_factor
        normal_acceleration = self._plan_parameters.max_acceleration_scaling_factor
        if fast:
            self._plan_parameters.max_velocity_scaling_factor = (
                self.lift_velocity_scaling
            )
            self._plan_parameters.max_acceleration_scaling_factor = (
                self.lift_acceleration_scaling
            )
        try:
            for attempt in range(1, self.plan_retries + 1):
                self._arm.set_start_state_to_current_state()
                monitor = self._moveit.get_planning_scene_monitor()
                with monitor.read_only() as scene:
                    goal_state = deepcopy(scene.current_state)
                    goal_state.set_joint_group_positions(
                        joint_model_group_name=self.arm_group,
                        position_values=np.asarray(positions, dtype=float),
                    )
                    goal_state.update()
                    if not scene.is_state_valid(goal_state, self.arm_group, False):
                        self.get_logger().warning(
                            f"{description}: joint goal is in collision"
                        )
                        return False

                if not self._arm.set_goal_state(robot_state=goal_state):
                    continue
                plan = self._arm.plan(self._plan_parameters)
                if not plan:
                    self.get_logger().warning(
                        f"{description}: no trajectory found "
                        f"(attempt {attempt}/{self.plan_retries}); retrying"
                    )
                    continue
                self.get_logger().info(
                    f"{description}: trajectory found; executing attempt {attempt}"
                )
                execution = self._moveit.execute(
                    self.arm_group,
                    plan.trajectory,
                    blocking=True,
                )
                if execution:
                    time.sleep(0.5)
                    return True
            return False
        finally:
            self._plan_parameters.max_velocity_scaling_factor = normal_velocity
            self._plan_parameters.max_acceleration_scaling_factor = normal_acceleration

    # Return the six measured arm-joint positions.
    def _current_arm_positions(self):
        names = (
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        )
        positions = [self._joint_position(name) for name in names]
        if any(position is None for position in positions):
            return None
        return positions

    # Rotate wrist 2 until the car is level or the next step is blocked.
    def _level_car_with_wrist(self):
        initial_positions = self._current_arm_positions()
        initial_tilt = self._car_tilt()
        if initial_positions is None or initial_tilt is None:
            self.get_logger().warning(
                "State 7: wrist levelling has no joint or car feedback"
            )
            return True
        self.get_logger().info(
            f"State 7 car tilt before wrist rotation: "
            f"{math.degrees(initial_tilt):.1f} deg"
        )
        initial_gap = self._car_pedestal_gap()
        if initial_gap is not None:
            self.get_logger().info(
                f"State 7 car-to-pedestal gap: {initial_gap * 1000.0:.1f} mm"
            )
            if initial_gap <= self.pedestal_contact_tolerance:
                self.get_logger().info(
                    "State 7: car is within the placement distance"
                )
                return True
        if initial_tilt <= self.place_target_tilt:
            return True

        probe_step = min(self.place_wrist_step, math.radians(6.0))
        direction = None
        current_tilt = initial_tilt

        for candidate_direction in (1.0, -1.0):
            candidate = list(initial_positions)
            candidate[4] += candidate_direction * probe_step
            if not -2.0 * math.pi <= candidate[4] <= 2.0 * math.pi:
                continue
            if not self._plan_and_execute_joint_positions(
                candidate,
                "State 7 Wrist Direction Check",
            ):
                continue

            measured_tilt = self._car_tilt()
            if (
                measured_tilt is not None
                and measured_tilt < initial_tilt - math.radians(0.5)
            ):
                direction = candidate_direction
                current_tilt = measured_tilt
                break

            if not self._plan_and_execute_joint_positions(
                initial_positions,
                "State 7 Restore Wrist",
            ):
                return False

        if direction is None:
            self.get_logger().warning(
                "State 7: wrist 2 cannot rotate farther toward level"
            )
            return True

        self.get_logger().info(
            f"State 7 wrist direction selected; car tilt is "
            f"{math.degrees(current_tilt):.1f} deg"
        )
        for step_number in range(1, self.place_wrist_max_steps + 1):
            if current_tilt <= self.place_target_tilt:
                self.get_logger().info(
                    "State 7: car reached the safe wrist release angle"
                )
                break
            gap = self._car_pedestal_gap()
            if gap is not None and gap <= self.pedestal_contact_tolerance:
                self.get_logger().info(
                    "State 7: car is within 5 mm of the pedestal"
                )
                break

            start_positions = self._current_arm_positions()
            if start_positions is None:
                return False
            step = min(
                self.place_wrist_step,
                max(current_tilt - self.place_target_tilt, math.radians(2.0)),
            )
            previous_tilt = current_tilt
            moved = False
            while step >= math.radians(1.0):
                positions = list(start_positions)
                positions[4] += direction * step
                if not -2.0 * math.pi <= positions[4] <= 2.0 * math.pi:
                    step *= 0.5
                    continue
                if self._plan_and_execute_joint_positions(
                    positions,
                    f"State 7 Lower Car {step_number}",
                ):
                    moved = True
                    break
                step *= 0.5

            if not moved:
                self.get_logger().info(
                    "State 7: next wrist rotation is blocked; releasing here"
                )
                break

            measured_tilt = self._car_tilt()
            if measured_tilt is None:
                return False
            current_tilt = measured_tilt
            gap = self._car_pedestal_gap()
            self.get_logger().info(
                f"State 7 car tilt: {math.degrees(current_tilt):.1f} deg, "
                f"gap: {gap * 1000.0:.1f} mm"
                if gap is not None
                else f"State 7 car tilt: {math.degrees(current_tilt):.1f} deg"
            )
            if current_tilt >= previous_tilt - math.radians(0.5):
                self.get_logger().info(
                    "State 7: wrist rotation no longer improves the angle"
                )
                break

        return True

    # Move the gripper straight up or down.
    def _move_vertical(
        self,
        start_pose,
        target_z,
        description,
        position_tolerance=None,
        orientation_tolerance=None,
        continuous=False,
    ):
        delta = target_z - start_pose.pose.position.z
        segments = (
            1
            if continuous
            else max(1, int(math.ceil(abs(delta) / self.vertical_step)))
        )
        normal_velocity = self._plan_parameters.max_velocity_scaling_factor
        normal_acceleration = self._plan_parameters.max_acceleration_scaling_factor
        if delta > 0.0:
            self._plan_parameters.max_velocity_scaling_factor = (
                self.lift_velocity_scaling
            )
            self._plan_parameters.max_acceleration_scaling_factor = (
                self.lift_acceleration_scaling
            )
        try:
            for index in range(1, segments + 1):
                waypoint = deepcopy(start_pose)
                waypoint.pose.position.z = (
                    start_pose.pose.position.z + delta * index / segments
                )
                if not self._plan_and_execute_pose(
                    waypoint,
                    f"{description} waypoint {index}/{segments}",
                    position_tolerance=position_tolerance,
                    orientation_tolerance=orientation_tolerance,
                ):
                    return False
            return True
        finally:
            self._plan_parameters.max_velocity_scaling_factor = normal_velocity
            self._plan_parameters.max_acceleration_scaling_factor = normal_acceleration

    # Send a position and effort goal to the gripper.
    def _command_gripper(
        self,
        position,
        require_contact=False,
        max_effort=None,
        result_timeout=None,
        feedback_timeout=5.0,
    ):
        if not self._gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warning("Gripper action server is not available")
            return False

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        if max_effort is None:
            max_effort = (
                self.gripper_grasp_effort
                if require_contact
                else self.gripper_effort
            )
        goal.command.max_effort = float(max_effort)

        goal_handle = self._wait_for_future(
            self._gripper_client.send_goal_async(goal),
            "the gripper goal response",
        )
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning("The gripper controller rejected the goal")
            return False

        result_future = goal_handle.get_result_async()
        timeout = self.future_timeout if result_timeout is None else result_timeout
        deadline = time.monotonic() + timeout
        feedback_reached_at = None
        while rclpy.ok() and not result_future.done():
            if not require_contact:
                actual_position = self._joint_position("left_finger_joint")
                if (
                    actual_position is not None
                    and abs(actual_position - position)
                    <= self.gripper_feedback_tolerance
                ):
                    if feedback_reached_at is None:
                        feedback_reached_at = time.monotonic()
                    elif time.monotonic() - feedback_reached_at >= 0.25:
                        self.get_logger().info(
                            "Gripper target verified from joint feedback: "
                            f"position={actual_position:.4f} m"
                        )
                        return True
                else:
                    feedback_reached_at = None
            if time.monotonic() >= deadline:
                self.get_logger().warning(
                    "Timed out waiting for the gripper result"
                )
                cancel_future = goal_handle.cancel_goal_async()
                cancel_deadline = time.monotonic() + 5.0
                while (
                    rclpy.ok()
                    and not cancel_future.done()
                    and time.monotonic() < cancel_deadline
                ):
                    time.sleep(0.05)
                if not cancel_future.done():
                    self.get_logger().warning(
                        "Timed out cancelling the gripper goal"
                    )
                else:
                    self.get_logger().info("Cancelled the timed-out gripper goal")
                time.sleep(0.25)
                return False
            time.sleep(0.05)

        if not result_future.done():
            return False
        try:
            wrapped_result = result_future.result()
        except Exception as error:
            self.get_logger().warning(f"The gripper result failed: {error}")
            return False
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning(
                f"Gripper action finished with status {wrapped_result.status}"
            )
            return False

        result = wrapped_result.result
        self.get_logger().info(
            "Gripper command succeeded: "
            f"position={result.position:.4f} m, "
            f"reached_goal={result.reached_goal}, stalled={result.stalled}"
        )
        if require_contact:
            if not result.stalled:
                self.get_logger().warning(
                    "Gripper reached the fully closed limit without stalling; "
                    "the car is not securely between the fingers"
                )
                return False
            if not (
                self.gripper_contact_min_position
                <= result.position
                <= self.gripper_contact_max_position
            ):
                self.get_logger().warning(
                    "Rejecting false gripper contact at "
                    f"{result.position:.4f} m; expected car contact in "
                    f"[{self.gripper_contact_min_position:.4f}, "
                    f"{self.gripper_contact_max_position:.4f}] m"
                )
                return False
        elif not result.reached_goal:
            # Contact can make the action report stalled while the jaws continue
            # moving. Check the measured joint before treating release as failed.
            feedback_deadline = time.monotonic() + feedback_timeout
            while rclpy.ok() and time.monotonic() < feedback_deadline:
                actual_position = self._joint_position("left_finger_joint")
                if (
                    actual_position is not None
                    and abs(actual_position - position)
                    <= self.gripper_feedback_tolerance
                ):
                    self.get_logger().info(
                        "Gripper target verified from joint feedback: "
                        f"position={actual_position:.4f} m"
                    )
                    return True
                time.sleep(0.05)
            self.get_logger().warning(
                f"Gripper stopped at {result.position:.4f} m instead of "
                f"reaching {position:.4f} m"
            )
            return False
        return True

    # Build the car collision object for MoveIt.
    def _car_object(self, object_id, pose, frame_id=None):
        collision = CollisionObject()
        collision.header.frame_id = frame_id or self.world_frame
        collision.id = object_id
        collision.operation = CollisionObject.ADD

        chassis_orientation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        parts = [
            (
                SolidPrimitive.BOX,
                [0.10, 0.05, 0.025],
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
            (
                SolidPrimitive.CYLINDER,
                [0.01, 0.02],
                (-0.025, 0.03, 0.0),
                (-1.5707, 0.0, 0.0),
            ),
            (
                SolidPrimitive.CYLINDER,
                [0.01, 0.02],
                (-0.025, -0.03, 0.0),
                (-1.5707, 0.0, 0.0),
            ),
            (
                SolidPrimitive.SPHERE,
                [0.01],
                (0.04, 0.0, -0.01),
                (0.0, 0.0, 0.0),
            ),
        ]
        for shape, dimensions, offset, part_rpy in parts:
            primitive = SolidPrimitive()
            primitive.type = shape
            primitive.dimensions = dimensions

            rotated_offset = self._rotate_by_quaternion(
                chassis_orientation,
                offset,
            )
            part_pose = Pose()
            part_pose.position.x = pose.position.x + rotated_offset[0]
            part_pose.position.y = pose.position.y + rotated_offset[1]
            part_pose.position.z = pose.position.z + rotated_offset[2]
            part_orientation = self._multiply_quaternions(
                chassis_orientation,
                self._quaternion_from_rpy(*part_rpy),
            )
            part_pose.orientation.x = part_orientation[0]
            part_pose.orientation.y = part_orientation[1]
            part_pose.orientation.z = part_orientation[2]
            part_pose.orientation.w = part_orientation[3]
            collision.primitives.append(primitive)
            collision.primitive_poses.append(part_pose)
        return collision

    # Apply a planning-scene change.
    def _apply_scene(self, scene, description):
        if not self._scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warning("MoveIt apply_planning_scene is unavailable")
            return False

        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self._wait_for_future(
            self._scene_client.call_async(request),
            description,
        )
        if response is None or not response.success:
            self.get_logger().warning(f"MoveIt rejected {description}")
            return False

        # Refresh the local planning scene.
        try:
            self._moveit.get_planning_scene_monitor().request_planning_scene_state(
                "/get_planning_scene"
            )
        except Exception as error:
            self.get_logger().warning(
                f"Could not immediately synchronise the planning scene: {error}"
            )
        time.sleep(0.25)
        return True

    # Allow the held car to touch the pedestal during placement.
    def _allow_car_pedestal_contact(self):
        if not self._scene_query_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warning("MoveIt planning scene is unavailable")
            return False

        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        response = self._wait_for_future(
            self._scene_query_client.call_async(request),
            "reading the allowed collision matrix",
        )
        if response is None:
            return False

        matrix = deepcopy(response.scene.allowed_collision_matrix)
        for name in (self.car_id, "pedestal_block"):
            if name in matrix.entry_names:
                continue
            for entry in matrix.entry_values:
                entry.enabled.append(False)
            matrix.entry_names.append(name)
            new_entry = AllowedCollisionEntry()
            new_entry.enabled = [False] * len(matrix.entry_names)
            matrix.entry_values.append(new_entry)

        car_index = matrix.entry_names.index(self.car_id)
        pedestal_index = matrix.entry_names.index("pedestal_block")
        matrix.entry_values[car_index].enabled[pedestal_index] = True
        matrix.entry_values[pedestal_index].enabled[car_index] = True

        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = matrix
        return self._apply_scene(scene, "allowing car contact with the pedestal")

    # Remove the loose car collision object.
    def _remove_car_from_world(self):
        remove = CollisionObject()
        remove.header.frame_id = self.world_frame
        remove.id = self.car_id
        remove.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [remove]
        if self._apply_scene(scene, "removing the car for grasp contact"):
            self._car_removed = True
            return True
        return False

    # Include the carried car in MoveIt's collision checks.
    def _add_carried_car_collision(self):
        relative_translation = self._car_translation_in_tcp()
        relative_pose = Pose()
        relative_pose.position.x = relative_translation[0]
        relative_pose.position.y = relative_translation[1]
        relative_pose.position.z = relative_translation[2]
        relative_pose.orientation.x = 1.0
        relative_pose.orientation.w = 0.0
        car_collision = self._car_object(
            self.car_id,
            relative_pose,
            self.tcp_link,
        )

        attached = AttachedCollisionObject()
        attached.link_name = self.tcp_link
        attached.touch_links = [
            "left_finger_link",
            "right_finger_link",
            "gripper_base_link",
            self.tcp_link,
        ]
        attached.object.header.frame_id = self.tcp_link
        attached.object.id = self.car_id
        attached.object.primitives = car_collision.primitives
        attached.object.primitive_poses = car_collision.primitive_poses
        attached.object.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        if self._apply_scene(scene, "adding carried-car collision geometry"):
            self._car_collision_attached = True
            return True
        return False

    # Remove the carried-car geometry when the physical jaws open.
    def _remove_carried_car_collision(self):
        detach = AttachedCollisionObject()
        detach.link_name = self.tcp_link
        detach.object.id = self.car_id
        detach.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [detach]
        remove_world = CollisionObject()
        remove_world.header.frame_id = self.world_frame
        remove_world.id = self.car_id
        remove_world.operation = CollisionObject.REMOVE
        scene.world.collision_objects = [remove_world]
        if self._apply_scene(scene, "removing carried-car collision geometry"):
            self._car_collision_attached = False
            self._car_removed = True
            return True
        return False

    # Restore the car collision object after a failed grasp.
    def _restore_car_collision(self):
        if not self._car_removed or self._car_collision_attached:
            return
        pose = Pose()
        pose.position.x = self.car_x
        pose.position.y = self.car_y
        pose.position.z = self.car_z
        pose.orientation.w = 1.0
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [self._car_object(self.car_id, pose)]
        if self._apply_scene(scene, "restoring the car collision object"):
            self._car_removed = False

    # Run the complete eight-state task.
    def run(self):
        self.initialise_moveit()
        if not self._wait_until_ready():
            return False

        # Copy the complete planning scene into MoveItPy.
        task_completed = False
        scene_received = False
        for attempt in range(1, 6):
            try:
                scene_received = (
                    self._moveit.get_planning_scene_monitor()
                    .request_planning_scene_state("/get_planning_scene")
                )
            except Exception as error:
                self.get_logger().warning(
                    "Planning-scene request failed "
                    f"({attempt}/5): {error}"
                )
            if scene_received is not False:
                break
            self.get_logger().warning(
                f"Planning scene unavailable ({attempt}/5); retrying"
            )
            time.sleep(0.5)
        if scene_received is False:
            self.get_logger().error(
                "MoveIt did not return the planning scene; refusing to move"
            )
            return False
        self.get_logger().info(
            "MoveIt planning scene synchronised (ground, car, and pedestal)"
        )
        time.sleep(0.25)

        grasp_x = self.car_x + self.tcp_x_offset
        grasp_pose = self._downward_pose(
            self.world_frame,
            grasp_x,
            self.car_y + self.tcp_y_offset,
            self.grasp_z,
        )
        pre_grasp = deepcopy(grasp_pose)
        pre_grasp.pose.position.z += self.pre_grasp_height

        try:
            # The target object may touch the open gripper during approach.
            if not self._remove_car_from_world():
                return False

            self._log_state("State 1: Pre-Grasp")
            if not self._plan_and_execute_joint_positions(
                self.pregrasp_joints,
                "State 1 Pre-Grasp",
            ):
                self.get_logger().warning(
                    "State 1 fixed branch failed; falling back to pose planning"
                )
                if not self._plan_and_execute_pose(
                    pre_grasp,
                    "State 1 Pre-Grasp fallback",
                    position_tolerance=0.020,
                    orientation_tolerance=math.radians(6.0),
                ):
                    return False
            if not self._centre_over_car(pre_grasp):
                return False

            self._log_state("State 2: Open Gripper")
            if not self._command_gripper(self.open_position):
                return False

            self._log_state("State 3: Grasp Approach")
            if not self._move_vertical(
                pre_grasp,
                grasp_pose.pose.position.z,
                "State 3 Grasp Approach",
                position_tolerance=0.015,
                orientation_tolerance=math.radians(6.0),
            ):
                return False

            self._log_state("State 4: Close Gripper")
            if not self._command_gripper(
                self.closed_position,
                require_contact=True,
            ):
                return False
            if not self._add_carried_car_collision():
                return False

            self._log_state("State 5: Post-Grasp (Lift)")
            lift_pose = deepcopy(grasp_pose)
            lift_pose.pose.position.z += self.lift_height
            if not self._move_vertical(
                grasp_pose,
                lift_pose.pose.position.z,
                "State 5 Post-Grasp Lift",
                continuous=True,
            ):
                return False
            if not self._grip_is_held("State 5 Post-Grasp Lift"):
                return False
            if not self._car_is_above(0.15, "State 5 Post-Grasp Lift"):
                return False

            self._log_state("State 6: Pedestal Approach")
            if not self._allow_car_pedestal_contact():
                return False
            if not self._plan_and_execute_joint_positions(
                self.pedestal_joints,
                "State 6 Pedestal Approach",
                fast=True,
            ):
                return False
            if not self._grip_is_held("State 6 Pedestal Approach"):
                return False
            if not self._car_is_above(0.70, "State 6 Pedestal Approach"):
                return False

            self._log_state("State 7: Place")
            if not self._level_car_with_wrist():
                return False

            coordinated_place = self._placement_pose()
            if not self._plan_and_execute_pose(
                coordinated_place,
                "State 7 Coordinated Centre and Lower",
                position_tolerance=0.015,
                orientation_tolerance=math.radians(6.0),
            ):
                self.get_logger().warning(
                    "State 7 coordinated lowering was blocked; "
                    "using the safe wrist release pose"
                )
            else:
                with self._car_pose_lock:
                    placed_pose = self._car_pose
                gap = self._car_pedestal_gap()
                if placed_pose is not None and gap is not None:
                    self.get_logger().info(
                        "State 7 coordinated car pose: "
                        f"({placed_pose[0]:.3f}, {placed_pose[1]:.3f}), "
                        f"gap={gap * 1000.0:.1f} mm"
                    )

            self._log_state("State 8: Open Gripper & Retreat")
            if not self._remove_carried_car_collision():
                return False

            if not self._command_gripper(
                self.gripper_settle_position,
                max_effort=self.gripper_release_effort,
                result_timeout=3.0,
            ):
                return False

            car_is_placed = self._wait_for_pedestal_contact()
            if not car_is_placed:
                second_settle_position = max(
                    self.gripper_settle_position,
                    self.open_position - 0.001,
                )
                if not self._command_gripper(
                    second_settle_position,
                    max_effort=self.gripper_release_effort,
                    result_timeout=3.0,
                ):
                    return False
                car_is_placed = self._wait_for_pedestal_contact()
            if not car_is_placed:
                self.get_logger().error(
                    "Refusing to release: car is not upright on the pedestal"
                )
                return False

            if not self._plan_and_execute_joint_positions(
                self.drive_clearance_joints,
                "State 8 Initial Tool Clearance",
                fast=True,
            ):
                return False

            gripper_opened = self._command_gripper(
                self.open_position,
                max_effort=self.gripper_release_effort,
                result_timeout=2.0,
                feedback_timeout=0.5,
            )
            if not gripper_opened:
                self.get_logger().info(
                    "Opening stalled; retrying with a longer timeout"
                )
                gripper_opened = self._command_gripper(
                    self.open_position,
                    max_effort=self.gripper_release_effort,
                    result_timeout=6.0,
                )
            if not gripper_opened:
                self.get_logger().error("The gripper did not open fully")
                return False

            clearance = self._finger_car_clearance()
            if clearance is None:
                self.get_logger().error(
                    "Cannot measure gripper-to-car clearance"
                )
                return False
            self.get_logger().info(
                "Finger collision-box clearance: "
                f"{clearance * 1000.0:.1f} mm"
            )
            if clearance < self.drive_clearance_distance:
                if not self._plan_and_execute_joint_positions(
                    self.release_retreat_joints,
                    "State 8 Additional Tool Clearance",
                    fast=True,
                ):
                    return False
                clearance = self._finger_car_clearance()
                if clearance is None:
                    return False
                self.get_logger().info(
                    "Finger collision-box clearance: "
                    f"{clearance * 1000.0:.1f} mm"
                )
            if clearance < self.drive_clearance_distance:
                self.get_logger().error(
                    "Refusing to drive: gripper collision boxes are too close"
                )
                return False

            trigger = Bool()
            trigger.data = True
            self._trigger_car_publisher.publish(trigger)
            self.get_logger().info(
                "Published True on /trigger_car after full gripper release "
                "and completed tool retreat"
            )

            home_succeeded = self._plan_and_execute_named(
                "home",
                "State 8 Retreat",
            )
            if not home_succeeded:
                return False
            self.get_logger().info("Pick-and-place pipeline completed successfully")
            task_completed = True
            return True
        finally:
            if not task_completed:
                self._restore_car_collision()

    # Stop the MoveIt executor thread.
    def shutdown_moveit(self):
        if self._moveit is not None:
            self._moveit.shutdown()
            time.sleep(0.25)


# Start the node and run the task.
def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    success = False
    try:
        success = node.run()
        if success:
            node.get_logger().info(
                "Pipeline is complete; keeping the ROS node alive until "
                "the launch is stopped"
            )
            while rclpy.ok():
                time.sleep(0.5)
    except KeyboardInterrupt:
        node.get_logger().warning("Pick-and-place pipeline interrupted")
    except Exception as error:
        node.get_logger().error(f"Pick-and-place pipeline failed: {error}")
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.shutdown_moveit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
