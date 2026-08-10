#!/usr/bin/env python3

import gc
import math
import threading
import time
from copy import deepcopy

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.collision_detection import CollisionRequest, CollisionResult
from moveit.core.robot_state import robotStateToRobotStateMsg
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


BLUE = "\033[1;94m"
RESET = "\033[0m"


# =============================================================================
# TASK WAYPOINTS - EDIT ROBOT POSITIONS HERE
# Positions are x, y, z in metres. Rotations are roll, pitch, yaw in radians.
# Change this block first when adjusting the arm path.
# Lower approach_x values move farther into the cell's left side.
# Higher approach_x values move toward the oven on the right.
# =============================================================================
TASK_WAYPOINTS = {
    # Point the closed fingertips toward the button at (0.131, -0.5595, 0.875).
    "button_approach": {
        "xyz": (0.090, -0.5595, 0.875),
        "rpy": (0.0, math.pi / 2.0, 0.0),
    },
    "button_press": {
        "xyz": (0.126, -0.5595, 0.875),
        "rpy": (0.0, math.pi / 2.0, 0.0),
    },
    "button_clear": {
        "xyz": (-0.100, -0.5595, 0.980),
        "rpy": (math.pi, 0.0, -math.pi / 2.0),
    },
    # Shelf tray pickup path.
    "fresh_tray": {
        "shelf_y": -0.492,
        "approach_x": -0.230,
        "edge_x": -0.317,
        "approach_height": 0.070,
        "lift_height": 0.090,
        "approach_rpy": (-math.pi / 2.0, 0.0, -math.pi / 2.0),
        "grasp_rpy": (-math.pi / 2.0, 0.0, math.pi / 2.0),
    },
    # Tray insertion path.
    "oven_tray": {
        "oven_y": -0.4395,
        "approach_x": 0.0825,
        "inside_x": 0.3325,
        "first_slot_z": 0.900,
        "slot_spacing": 0.100,
        "rpy": (-math.pi / 2.0, 0.0, -math.pi / 2.0),
    },
    # Oven door handle path.
    "oven_door": {
        "hinge_xyz": (0.145, -0.267, 1.0425),
        "handle_xy": (0.2925, -0.030),
        "closed_yaw": -math.pi / 2.0,
        "opening_angles_deg": (-25.0, -50.0, -75.0, -90.0),
        "closing_angles_deg": (-65.0, -40.0, -20.0, 0.0),
        "approach_clearance": 0.070,
        "retreat_clearance": 0.080,
    },
    # Baked-item pickup path.
    "oven_items": {
        "y_positions": (-0.3495, -0.4395, -0.5295),
        "approach_x": 0.0825,
        "pickup_x": 0.340,
        "approach_z": 1.020,
        "grasp_tcp_z": 0.884,
        "lift_z": 1.080,
        "rpy": (math.pi, 0.0, -math.pi / 2.0),
    },
    # Cooling-tray placement path.
    "cooling_tray": {
        "x": -0.407,
        "y_positions": (-0.582, -0.492, -0.402),
        "approach_z": 1.520,
        "place_z": 1.385,
        "rpy": (math.pi, 0.0, 0.0),
    },
}


class BakeryTaskNode(Node):
    # Run the three-item bakery task.

    def __init__(self):
        # Create the task interfaces and settings.
        super().__init__("bakery_task_node")
        self.declare_settings()

        self.world_frame = self.read_string("world_frame")
        self.arm_group = self.read_string("arm_group")
        self.tcp_link = self.read_string("tcp_link")
        self.sequence = [
            name.strip()
            for name in self.read_string("object_sequence").split(",")
            if name.strip()
        ]
        self.plan_retries = self.read_int("plan_retries")
        self.future_timeout = self.read_float("future_timeout")
        self.startup_timeout = self.read_float("startup_timeout")
        self.open_position = self.read_float("gripper_open_position")
        self.closed_position = self.read_float("gripper_closed_position")
        self.gripper_effort = self.read_float("gripper_effort")
        self.automatic_bake_delay = self.read_float("automatic_bake_delay")
        self.waypoints = deepcopy(TASK_WAYPOINTS)
        self.arm_joint_names = (
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        )

        catalog_path = (
            get_package_share_directory("auto_cake_feeder")
            + "/config/bakery_items.yaml"
        )
        with open(catalog_path, encoding="utf-8") as catalog_file:
            self.items = yaml.safe_load(catalog_file)["items"]

        unknown = set(self.sequence) - set(self.items)
        if len(self.sequence) != 3 or len(set(self.sequence)) != 3 or unknown:
            raise ValueError(
                "object_sequence must contain three unique bakery item names; "
                f"unknown names: {sorted(unknown)}"
            )

        self.controller_client = self.create_client(
            ListControllers,
            "/controller_manager/list_controllers",
        )
        self.scene_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )
        self.state_validity_client = self.create_client(
            GetStateValidity,
            "/check_state_validity",
        )
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            "/gripper_controller/gripper_cmd",
        )
        self.door_collision_publisher = self.create_publisher(
            Bool,
            "/door_collision_enabled",
            10,
        )
        self.button_contact_publisher = self.create_publisher(
            Bool,
            "/button_contact_enabled",
            10,
        )
        self.shelf_access_publisher = self.create_publisher(
            Bool,
            "/shelf_access_enabled",
            10,
        )
        self.create_service(Trigger, "/bake_complete", self.on_bake_complete)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_subscription(
            JointState,
            "/oven/joint_state",
            self.on_oven_state,
            10,
        )
        self.create_subscription(
            TFMessage,
            "/world/empty/dynamic_pose/info",
            self.on_dynamic_pose,
            10,
        )

        self.data_lock = threading.Lock()
        self.joint_positions = {}
        self.oven_positions = {}
        self.model_poses = {}
        self.joint_state_ready = threading.Event()
        self.oven_state_ready = threading.Event()
        self.bake_complete = threading.Event()
        self.moveit = None
        self.arm = None
        self.plan_parameters = None

    def declare_settings(self):
        # Declare the settings that can be changed from launch.
        settings = {
            "world_frame": "world",
            "arm_group": "ur_manipulator",
            "tcp_link": "gripper_tcp",
            "object_sequence": "croissant,cheese_roll,cupcake",
            "plan_retries": 4,
            "planning_attempts": 5,
            "planning_time": 8.0,
            "velocity_scaling": 0.30,
            "acceleration_scaling": 0.22,
            "future_timeout": 35.0,
            "startup_timeout": 90.0,
            "automatic_bake_delay": 0.0,
            "gripper_open_position": 0.035,
            "gripper_closed_position": 0.0,
            "gripper_effort": 50.0,
            "gripper_open_tolerance": 0.002,
        }
        for name, value in settings.items():
            self.declare_parameter(name, value)

    def read_string(self, name):
        # Read a text setting.
        return self.get_parameter(name).get_parameter_value().string_value

    def read_float(self, name):
        # Read a decimal setting.
        return self.get_parameter(name).get_parameter_value().double_value

    def read_int(self, name):
        # Read a whole-number setting.
        return self.get_parameter(name).get_parameter_value().integer_value

    def log_state(self, message):
        # Print a blue state message.
        self.get_logger().info(f"{BLUE}{message}{RESET}")

    def on_bake_complete(self, request, response):
        # Accept the bake-complete signal.
        del request
        self.bake_complete.set()
        response.success = True
        response.message = "Bake-complete signal received"
        self.log_state("Bake-complete signal received")
        return response

    def on_joint_state(self, message):
        # Store the latest robot joints.
        with self.data_lock:
            self.joint_positions.update(zip(message.name, message.position))
        required = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
            "left_finger_joint",
        }
        if required.issubset(self.joint_positions):
            self.joint_state_ready.set()

    def on_oven_state(self, message):
        # Store the oven door and button positions.
        with self.data_lock:
            self.oven_positions.update(zip(message.name, message.position))
        if "oven_door_joint" in self.oven_positions:
            self.oven_state_ready.set()

    def on_dynamic_pose(self, message):
        # Store Gazebo model poses.
        with self.data_lock:
            for transform in message.transforms:
                point = transform.transform.translation
                rotation = transform.transform.rotation
                self.model_poses[transform.child_frame_id] = (
                    point.x,
                    point.y,
                    point.z,
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                )

    def model_pose(self, model_name):
        # Return a model pose using Gazebo's model name.
        with self.data_lock:
            direct = self.model_poses.get(model_name)
            if direct is not None:
                return direct
            for frame_name, pose in self.model_poses.items():
                if frame_name.startswith(model_name + "/"):
                    return pose
        return None

    def oven_position(self, joint_name):
        # Return one oven joint position.
        with self.data_lock:
            return self.oven_positions.get(joint_name)

    def initialise_moveit(self):
        # Create the MoveIt planning interface.
        self.moveit = MoveItPy(
            node_name="bakery_task_moveit_py",
            provide_planning_service=False,
        )
        self.arm = self.moveit.get_planning_component(self.arm_group)
        self.plan_parameters = PlanRequestParameters(self.moveit)
        self.plan_parameters.planning_pipeline = "ompl"
        self.plan_parameters.planner_id = "RRTConnect"
        self.plan_parameters.planning_attempts = self.read_int("planning_attempts")
        self.plan_parameters.planning_time = self.read_float("planning_time")
        self.plan_parameters.max_velocity_scaling_factor = self.read_float(
            "velocity_scaling"
        )
        self.plan_parameters.max_acceleration_scaling_factor = self.read_float(
            "acceleration_scaling"
        )

    def wait_for_future(self, future, description, timeout=None):
        # Wait for one ROS action or service response.
        deadline = time.monotonic() + (timeout or self.future_timeout)
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                self.get_logger().warning(f"Timed out waiting for {description}")
                return None
            time.sleep(0.05)
        try:
            return future.result() if future.done() else None
        except Exception as error:
            self.get_logger().warning(f"{description} failed: {error}")
            return None

    def wait_until_ready(self):
        # Wait for controllers and sensor feedback.
        required = {
            "arm_controller",
            "gripper_controller",
            "joint_state_broadcaster",
        }
        deadline = time.monotonic() + self.startup_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if not self.controller_client.wait_for_service(timeout_sec=1.0):
                continue
            result = self.wait_for_future(
                self.controller_client.call_async(ListControllers.Request()),
                "controller list",
                5.0,
            )
            if result is None:
                continue
            active = {
                controller.name
                for controller in result.controller
                if controller.state == "active"
            }
            if (
                required.issubset(active)
                and self.joint_state_ready.wait(1.0)
                and self.oven_state_ready.wait(1.0)
                and self.gripper_client.wait_for_server(timeout_sec=1.0)
            ):
                self.get_logger().info("Robot, gripper and oven feedback are ready")
                return True
            time.sleep(0.5)
        self.get_logger().error("The simulation was not ready before the timeout")
        return False

    @staticmethod
    def quaternion_from_rpy(roll, pitch, yaw):
        # Convert roll, pitch and yaw to a quaternion.
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def pose_from_rpy(self, x, y, z, roll, pitch, yaw):
        # Create a world-frame pose.
        target = PoseStamped()
        target.header.frame_id = self.world_frame
        target.pose.position.x = float(x)
        target.pose.position.y = float(y)
        target.pose.position.z = float(z)
        quaternion = self.quaternion_from_rpy(roll, pitch, yaw)
        target.pose.orientation.x = quaternion[0]
        target.pose.orientation.y = quaternion[1]
        target.pose.orientation.z = quaternion[2]
        target.pose.orientation.w = quaternion[3]
        return target

    def waypoint_pose(self, name):
        # Create a pose from the editable waypoint table.
        waypoint = self.waypoints[name]
        return self.pose_from_rpy(*waypoint["xyz"], *waypoint["rpy"])

    def wait_for_arm_goal(self, goal_state, timeout=15.0, tolerance=0.04):
        # Confirm that the physical arm reached the planned joint positions.
        goal_positions = goal_state.get_joint_group_positions(self.arm_group)
        deadline = time.monotonic() + timeout
        maximum_error = float("inf")
        while rclpy.ok() and time.monotonic() < deadline:
            with self.data_lock:
                actual = [self.joint_positions.get(name) for name in self.arm_joint_names]
            if all(position is not None for position in actual):
                errors = [
                    abs(math.atan2(math.sin(goal - current), math.cos(goal - current)))
                    for current, goal in zip(actual, goal_positions)
                ]
                maximum_error = max(errors)
                if maximum_error <= tolerance:
                    return True
            time.sleep(0.1)
        self.get_logger().warning(
            f"Arm did not settle at the goal; maximum joint error "
            f"{math.degrees(maximum_error):.1f} degrees"
        )
        return False

    def find_goal_state(self, target, selection_index=0):
        # Find one collision-free inverse-kinematics solution.
        ik_solutions = 0
        colliding_solutions = 0
        self_colliding_solutions = 0
        valid_candidates = []
        self.last_invalid_state_msg = None
        monitor = self.moveit.get_planning_scene_monitor()
        with monitor.read_only() as scene:
            seed = deepcopy(scene.current_state)
            joint_group = seed.robot_model.get_joint_model_group(self.arm_group)
            current_positions = seed.get_joint_group_positions(self.arm_group)
            for candidate_index in range(30):
                candidate = deepcopy(seed)
                if candidate_index:
                    candidate.set_to_random_positions(joint_group)
                candidate.update()
                if not candidate.set_from_ik(
                    self.arm_group,
                    target.pose,
                    self.tcp_link,
                    timeout=0.08,
                ):
                    continue
                ik_solutions += 1
                candidate.update()
                if not scene.is_state_colliding(candidate, self.arm_group, False):
                    candidate_positions = candidate.get_joint_group_positions(
                        self.arm_group
                    )
                    distance = sum(
                        abs(math.atan2(math.sin(goal - start), math.cos(goal - start)))
                        for start, goal in zip(current_positions, candidate_positions)
                    )
                    valid_candidates.append((distance, candidate))
                else:
                    colliding_solutions += 1
                    request = CollisionRequest()
                    result = CollisionResult()
                    scene.check_self_collision(request, result, candidate)
                    if result.collision:
                        self_colliding_solutions += 1
                    elif self.last_invalid_state_msg is None:
                        self.last_invalid_state_msg = robotStateToRobotStateMsg(candidate)
        self.last_ik_counts = (
            ik_solutions,
            colliding_solutions,
            self_colliding_solutions,
        )
        if not valid_candidates:
            return None
        valid_candidates.sort(key=lambda item: item[0])
        index = min(selection_index, len(valid_candidates) - 1)
        return valid_candidates[index][1]

    def describe_last_collision(self):
        # Return the collision pairs for the last rejected goal state.
        if self.last_invalid_state_msg is None:
            return []
        if not self.state_validity_client.wait_for_service(timeout_sec=1.0):
            return []
        request = GetStateValidity.Request()
        request.robot_state = self.last_invalid_state_msg
        request.group_name = self.arm_group
        response = self.wait_for_future(
            self.state_validity_client.call_async(request),
            "State-validity check",
            timeout=2.0,
        )
        if response is None:
            return []
        return sorted({
            f"{contact.contact_body_1} <-> {contact.contact_body_2}"
            for contact in response.contacts
        })

    def move_to_pose(
        self,
        target,
        description,
        use_alternate_ik=True,
        settle_tolerance=0.04,
        require_settle=True,
    ):
        # Plan, check and execute one arm movement.
        for attempt in range(1, self.plan_retries + 1):
            self.arm.set_start_state_to_current_state()
            selection_index = attempt - 1 if use_alternate_ik else 0
            goal_state = self.find_goal_state(target, selection_index)
            if goal_state is None:
                (
                    ik_solutions,
                    colliding_solutions,
                    self_colliding_solutions,
                ) = self.last_ik_counts
                collision_text = ""
                if attempt == 1:
                    pairs = self.describe_last_collision()
                    if pairs:
                        collision_text = f"; contacts: {', '.join(pairs)}"
                self.get_logger().warning(
                    f"{description}: no valid IK solution "
                    f"({ik_solutions} IK, {colliding_solutions} colliding, "
                    f"{self_colliding_solutions} self-colliding) "
                    f"({attempt}/{self.plan_retries}); retrying{collision_text}"
                )
                continue
            if not self.arm.set_goal_state(robot_state=goal_state):
                self.get_logger().warning(
                    f"{description}: goal rejected "
                    f"({attempt}/{self.plan_retries}); retrying"
                )
                continue
            plan = self.arm.plan(self.plan_parameters)
            if not plan:
                code = getattr(plan.error_code, "val", "unknown")
                self.get_logger().warning(
                    f"{description}: planning failed with code {code} "
                    f"({attempt}/{self.plan_retries}); retrying"
                )
                continue
            self.get_logger().info(
                f"{description}: trajectory found; executing attempt {attempt}"
            )
            result = self.moveit.execute(
                self.arm_group,
                plan.trajectory,
                blocking=True,
            )
            if result:
                if not require_settle:
                    return True
                if self.wait_for_arm_goal(
                    goal_state,
                    tolerance=settle_tolerance,
                ):
                    return True
            self.get_logger().warning(
                f"{description}: execution failed "
                f"({attempt}/{self.plan_retries}); replanning"
            )
        self.get_logger().error(f"{description}: all planning attempts failed")
        return False

    def move_home(self):
        # Plan and move to the home state.
        for attempt in range(1, self.plan_retries + 1):
            self.arm.set_start_state_to_current_state()
            if not self.arm.set_goal_state(configuration_name="home"):
                continue
            plan = self.arm.plan(self.plan_parameters)
            if plan and self.moveit.execute(
                self.arm_group,
                plan.trajectory,
                blocking=True,
            ):
                return True
            self.get_logger().warning(
                f"Home move failed ({attempt}/{self.plan_retries}); retrying"
            )
        return False

    def command_gripper(self, position, description):
        # Send a position and effort to the gripper.
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = self.gripper_effort
        goal_handle = self.wait_for_future(
            self.gripper_client.send_goal_async(goal),
            description + " goal",
        )
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning(f"{description}: gripper goal was rejected")
            return False
        wrapped = self.wait_for_future(
            goal_handle.get_result_async(),
            description + " result",
            12.0,
        )
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning(f"{description}: gripper action failed")
            return False
        result = wrapped.result
        self.get_logger().info(
            f"{description}: position={result.position:.4f} m, "
            f"reached={result.reached_goal}, stalled={result.stalled}"
        )
        if position >= self.open_position - 0.0005:
            tolerance = self.read_float("gripper_open_tolerance")
            return result.reached_goal or abs(result.position - position) <= tolerance
        return result.reached_goal or result.stalled

    def set_door_collision(self, enabled):
        # Tell MoveIt whether to avoid the moving door.
        message = Bool()
        message.data = enabled
        for unused in range(3):
            self.door_collision_publisher.publish(message)
            time.sleep(0.1)
        time.sleep(1.0)
        self.refresh_planning_scene()

    def set_button_contact(self, enabled):
        # Allow physical contact only while pressing the button.
        message = Bool()
        message.data = enabled
        for unused in range(3):
            self.button_contact_publisher.publish(message)
            time.sleep(0.1)
        time.sleep(1.0)
        self.refresh_planning_scene()

    def set_shelf_access(self, enabled):
        # Allow the gripper to enter the physical shelf opening.
        message = Bool()
        message.data = enabled
        for unused in range(3):
            self.shelf_access_publisher.publish(message)
            time.sleep(0.1)
        time.sleep(1.0)
        self.refresh_planning_scene()

    def refresh_planning_scene(self):
        # Copy the latest move_group scene into MoveItPy.
        if self.moveit is None:
            return
        try:
            self.moveit.get_planning_scene_monitor().request_planning_scene_state(
                "/get_planning_scene"
            )
        except Exception as error:
            self.get_logger().warning(f"Planning-scene refresh failed: {error}")
        time.sleep(0.25)

    def make_box(self, object_id, dimensions, pose, frame_id):
        # Create a box collision object.
        collision = CollisionObject()
        collision.header.frame_id = frame_id
        collision.id = object_id
        collision.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(dimensions)
        collision.primitives.append(primitive)
        collision.primitive_poses.append(pose)
        return collision

    def apply_scene(self, scene, description):
        # Apply one MoveIt collision-scene update.
        if not self.scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warning("MoveIt planning-scene service is unavailable")
            return False
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self.wait_for_future(
            self.scene_client.call_async(request),
            description,
        )
        if response is None or not response.success:
            self.get_logger().warning(f"MoveIt rejected {description}")
            return False
        try:
            self.moveit.get_planning_scene_monitor().request_planning_scene_state(
                "/get_planning_scene"
            )
        except Exception as error:
            self.get_logger().warning(f"Scene refresh failed: {error}")
        time.sleep(0.25)
        return True

    def attach_box(self, object_id, dimensions, relative_xyz, relative_rpy):
        # Add the carried object to the robot collision model.
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = relative_xyz
        quaternion = self.quaternion_from_rpy(*relative_rpy)
        pose.orientation.x, pose.orientation.y = quaternion[0], quaternion[1]
        pose.orientation.z, pose.orientation.w = quaternion[2], quaternion[3]
        attached = AttachedCollisionObject()
        attached.link_name = self.tcp_link
        attached.touch_links = [
            "gripper_tcp",
            "gripper_base_link",
            "left_finger_link",
            "right_finger_link",
        ]
        attached.object = self.make_box(
            object_id,
            dimensions,
            pose,
            self.tcp_link,
        )
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        return self.apply_scene(scene, f"attaching {object_id} to the collision model")

    def detach_box(self, object_id):
        # Remove a carried object from the robot collision model.
        attached = AttachedCollisionObject()
        attached.link_name = self.tcp_link
        attached.object.id = object_id
        attached.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        return self.apply_scene(scene, f"detaching {object_id} from the collision model")

    def verify_lift(self, model_name, starting_pose, minimum_rise=0.025):
        # Check that Gazebo shows the object moving upward.
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < deadline:
            current = self.model_pose(model_name)
            if current is not None and starting_pose is not None:
                if current[2] >= starting_pose[2] + minimum_rise:
                    self.get_logger().info(
                        f"{model_name} physical lift verified: "
                        f"{(current[2] - starting_pose[2]) * 1000.0:.1f} mm"
                    )
                    return True
            time.sleep(0.1)
        self.get_logger().error(f"{model_name} did not move with the gripper")
        return False

    def press_oven_start(self, cycle):
        # Press and release the oven button.
        self.log_state(f"Cycle {cycle}: State 1 - Press Oven Start")
        if not self.command_gripper(self.closed_position, "Close for oven button"):
            return False
        pre = self.waypoint_pose("button_approach")
        press = self.waypoint_pose("button_press")
        clear = self.waypoint_pose("button_clear")
        normal_velocity = self.plan_parameters.max_velocity_scaling_factor
        normal_acceleration = self.plan_parameters.max_acceleration_scaling_factor
        self.set_door_collision(False)
        self.set_button_contact(True)
        try:
            if not self.move_to_pose(clear, "Move above the oven button"):
                return False
            if not self.move_to_pose(
                pre,
                "Oven button approach",
                require_settle=False,
            ):
                return False
            # Move slowly for reliable physical contact with the small button.
            self.plan_parameters.max_velocity_scaling_factor = min(
                normal_velocity, 0.15
            )
            self.plan_parameters.max_acceleration_scaling_factor = min(
                normal_acceleration, 0.15
            )
            if not self.move_to_pose(
                press,
                "Oven button press",
                use_alternate_ik=False,
                require_settle=False,
            ):
                return False
            button_position = self.oven_position("start_button_joint")
            if button_position is None or button_position < 0.003:
                self.get_logger().warning(
                    "The button displacement was not confirmed; pressing 2 mm farther"
                )
                deeper = deepcopy(press)
                deeper.pose.position.x += 0.002
                if not self.move_to_pose(
                    deeper,
                    "Oven button press correction",
                    use_alternate_ik=False,
                    require_settle=False,
                ):
                    return False
                time.sleep(0.2)
                button_position = self.oven_position("start_button_joint")
            if button_position is None or button_position < 0.003:
                self.get_logger().error("The oven button did not move at least 3 mm")
                return False
            self.get_logger().info(
                f"Oven button press verified: {button_position * 1000.0:.1f} mm"
            )
            self.plan_parameters.max_velocity_scaling_factor = normal_velocity
            self.plan_parameters.max_acceleration_scaling_factor = normal_acceleration
            if not self.move_to_pose(
                pre,
                "Oven button retreat",
                settle_tolerance=0.065,
            ):
                return False
            if not self.move_to_pose(clear, "Clear the oven after button press"):
                return False
            return True
        finally:
            self.plan_parameters.max_velocity_scaling_factor = normal_velocity
            self.plan_parameters.max_acceleration_scaling_factor = normal_acceleration
            self.set_button_contact(False)
            self.set_door_collision(True)

    def grasp_fresh_tray(self, cycle):
        # Pick the next tray from the shelf.
        self.log_state(f"Cycle {cycle}: State 2 - Grasp Fresh Tray")
        waypoint = self.waypoints["fresh_tray"]
        tray_name = f"fresh_tray_{4 - cycle}"
        tray_z = 0.8005 + (3 - cycle) * 0.0032
        pre = self.pose_from_rpy(
            waypoint["approach_x"],
            waypoint["shelf_y"],
            tray_z + waypoint["approach_height"],
            *waypoint["approach_rpy"],
        )
        edge = self.pose_from_rpy(
            waypoint["edge_x"],
            waypoint["shelf_y"],
            tray_z,
            *waypoint["grasp_rpy"],
        )
        start_pose = self.model_pose(tray_name)
        self.set_shelf_access(True)
        try:
            if not self.move_to_pose(pre, "Fresh tray approach"):
                return None
            if not self.command_gripper(self.open_position, "Open for tray"):
                return None
            if not self.move_to_pose(edge, "Fresh tray edge"):
                return None
            if not self.command_gripper(self.closed_position, "Grip fresh tray"):
                return None
            if not self.attach_box(
                tray_name + "_carried",
                (0.320, 0.180, 0.003),
                (0.0, 0.0, 0.090),
                (math.pi / 2, 0.0, 0.0),
            ):
                return None
            lift = deepcopy(edge)
            lift.pose.position.z += waypoint["lift_height"]
            if not self.move_to_pose(lift, "Lift fresh tray"):
                return None
            if not self.verify_lift(tray_name, start_pose):
                return None
            return tray_name, lift
        finally:
            self.set_shelf_access(False)

    def insert_tray(self, cycle, tray_name, start_pose):
        # Insert and release the tray in the oven.
        self.log_state(f"Cycle {cycle}: State 3 - Insert Tray into Oven Shelf")
        waypoint = self.waypoints["oven_tray"]
        slot_z = waypoint["first_slot_z"] + (cycle - 1) * waypoint["slot_spacing"]
        front = self.pose_from_rpy(
            waypoint["approach_x"],
            waypoint["oven_y"],
            slot_z,
            *waypoint["rpy"],
        )
        inside = self.pose_from_rpy(
            waypoint["inside_x"],
            waypoint["oven_y"],
            slot_z,
            *waypoint["rpy"],
        )
        if not self.move_to_pose(front, "Carry tray to oven"):
            return False
        if not self.move_to_pose(inside, "Insert tray into oven"):
            return False
        if not self.detach_box(tray_name + "_carried"):
            return False
        if not self.command_gripper(self.open_position, "Release tray"):
            return False
        released_pose = self.model_pose(tray_name)
        if released_pose is not None and start_pose is not None:
            distance = math.hypot(
                released_pose[0] - waypoint["inside_x"],
                released_pose[1] - waypoint["oven_y"],
            )
            if distance > 0.10:
                self.get_logger().warning(
                    f"{tray_name} is {distance * 1000.0:.1f} mm from its oven target"
                )
        retreat = deepcopy(front)
        return self.move_to_pose(retreat, "Retreat after tray insertion")

    def wait_for_baking(self, cycle):
        # Wait for the external bake-complete signal.
        self.log_state(f"Cycle {cycle}: State 4 - Wait for Baking Complete")
        if self.automatic_bake_delay > 0.0:
            self.get_logger().info(
                f"Demo mode: baking for {self.automatic_bake_delay:.1f} seconds"
            )
            deadline = time.monotonic() + self.automatic_bake_delay
            while rclpy.ok() and time.monotonic() < deadline:
                time.sleep(0.1)
            return rclpy.ok()
        self.get_logger().info(
            'Signal completion with: ros2 service call /bake_complete '
            'std_srvs/srv/Trigger "{}"'
        )
        while rclpy.ok() and not self.bake_complete.wait(0.25):
            pass
        self.bake_complete.clear()
        return rclpy.ok()

    def door_handle_pose(self, angle):
        # Calculate the door-handle pose at one angle.
        waypoint = self.waypoints["oven_door"]
        hinge_x, hinge_y, hinge_z = waypoint["hinge_xyz"]
        local_x, local_y = waypoint["handle_xy"]
        world_yaw = waypoint["closed_yaw"] + angle
        x = hinge_x + local_x * math.cos(world_yaw) - local_y * math.sin(world_yaw)
        y = hinge_y + local_x * math.sin(world_yaw) + local_y * math.cos(world_yaw)
        return self.pose_from_rpy(
            x, y, hinge_z, -math.pi / 2, 0.0, world_yaw
        )

    def move_door(self, cycle, opening):
        # Open or close the physical oven door.
        action = "Open" if opening else "Close"
        state = 5 if opening else 8
        self.log_state(f"Cycle {cycle}: State {state} - {action} Oven Door")
        waypoint = self.waypoints["oven_door"]
        start_angle = 0.0 if opening else -math.pi / 2
        approach = self.door_handle_pose(start_angle)
        start_yaw = waypoint["closed_yaw"] + start_angle
        approach.pose.position.x += waypoint["approach_clearance"] * math.sin(start_yaw)
        approach.pose.position.y -= waypoint["approach_clearance"] * math.cos(start_yaw)
        if not self.command_gripper(self.open_position, f"Open for door {action.lower()}"):
            return False
        if not self.move_to_pose(approach, f"Door {action.lower()} approach"):
            return False
        if not self.move_to_pose(self.door_handle_pose(start_angle), "Reach door handle"):
            return False
        if not self.command_gripper(0.010, "Grip door handle"):
            return False
        self.set_door_collision(False)
        angles = (
            waypoint["opening_angles_deg"]
            if opening
            else waypoint["closing_angles_deg"]
        )
        try:
            for degrees in angles:
                if not self.move_to_pose(
                    self.door_handle_pose(math.radians(degrees)),
                    f"Door {action.lower()} {abs(degrees)} degrees",
                ):
                    return False
            actual = self.oven_position("oven_door_joint")
            if actual is None:
                self.get_logger().warning("Oven door angle is unavailable")
            elif opening and actual > -1.20:
                self.get_logger().warning(
                    f"Door only opened to {math.degrees(actual):.1f} degrees"
                )
                return False
            elif not opening and actual < -0.15:
                self.get_logger().warning(
                    f"Door only closed to {math.degrees(actual):.1f} degrees"
                )
                return False
            if not self.command_gripper(self.open_position, "Release door handle"):
                return False
            retreat = deepcopy(self.door_handle_pose(math.radians(angles[-1])))
            final_yaw = waypoint["closed_yaw"] + math.radians(angles[-1])
            retreat.pose.position.x += waypoint["retreat_clearance"] * math.sin(final_yaw)
            retreat.pose.position.y -= waypoint["retreat_clearance"] * math.cos(final_yaw)
            return self.move_to_pose(retreat, "Retreat from door")
        finally:
            self.set_door_collision(True)

    def remove_bakery_item(self, cycle, item_name):
        # Pick one baked item from the oven.
        self.log_state(f"Cycle {cycle}: State 6 - Remove {item_name}")
        waypoint = self.waypoints["oven_items"]
        dimensions = tuple(float(value) for value in self.items[item_name]["dimensions_m"])
        item_x = waypoint["pickup_x"]
        item_y = waypoint["y_positions"][cycle - 1]
        rpy = waypoint["rpy"]
        pre = self.pose_from_rpy(
            waypoint["approach_x"], item_y, waypoint["approach_z"], *rpy
        )
        above = self.pose_from_rpy(
            item_x, item_y, waypoint["approach_z"], *rpy
        )
        grasp = self.pose_from_rpy(
            item_x, item_y, waypoint["grasp_tcp_z"], *rpy
        )
        start_pose = self.model_pose(item_name)
        if not self.command_gripper(self.open_position, "Open for bakery item"):
            return None
        for target, label in (
            (pre, "Oven item approach"),
            (above, "Move above bakery item"),
            (grasp, "Lower around bakery item"),
        ):
            if not self.move_to_pose(target, label):
                return None
        if not self.command_gripper(self.closed_position, "Grip bakery item"):
            return None
        if not self.attach_box(
            item_name + "_carried",
            dimensions,
            (0.0, 0.0, -dimensions[2] / 2.0),
            (0.0, 0.0, 0.0),
        ):
            return None
        lift = deepcopy(grasp)
        lift.pose.position.z = waypoint["lift_z"]
        if not self.move_to_pose(lift, "Lift bakery item from oven"):
            return None
        if not self.verify_lift(item_name, start_pose):
            return None
        return dimensions, lift

    def place_on_cooling_tray(self, cycle, item_name, dimensions):
        # Place the item on the cooling tray.
        self.log_state(f"Cycle {cycle}: State 7 - Place on Cooling Tray")
        waypoint = self.waypoints["cooling_tray"]
        target_x = waypoint["x"]
        target_y = waypoint["y_positions"][cycle - 1]
        rpy = waypoint["rpy"]
        above = self.pose_from_rpy(
            target_x, target_y, waypoint["approach_z"], *rpy
        )
        place = self.pose_from_rpy(
            target_x, target_y, waypoint["place_z"], *rpy
        )
        if not self.move_to_pose(above, "Cooling tray approach"):
            return False
        if not self.move_to_pose(place, "Lower item onto cooling tray"):
            return False
        if not self.detach_box(item_name + "_carried"):
            return False
        if not self.command_gripper(self.open_position, "Release bakery item"):
            return False
        time.sleep(0.7)
        physical = self.model_pose(item_name)
        if physical is not None:
            error = math.hypot(physical[0] - target_x, physical[1] - target_y)
            if error > 0.060 or physical[2] < 1.36:
                self.get_logger().warning(
                    f"{item_name} release position is outside the cooling target"
                )
                return False
        retreat = deepcopy(above)
        return self.move_to_pose(retreat, "Retreat from cooling tray")

    def run_cycle(self, cycle, item_name):
        # Run all eight states for one item.
        # Press the oven button.
        if not self.press_oven_start(cycle):
            return False
        # Pick and insert a fresh tray.
        tray_result = self.grasp_fresh_tray(cycle)
        if tray_result is None:
            return False
        tray_name, tray_pose = tray_result
        if not self.insert_tray(cycle, tray_name, tray_pose):
            return False
        # Wait for baking and open the door.
        if not self.wait_for_baking(cycle):
            return False
        if not self.move_door(cycle, True):
            return False
        # Move the baked item to the cooling tray.
        item_result = self.remove_bakery_item(cycle, item_name)
        if item_result is None:
            return False
        dimensions, unused_pose = item_result
        del unused_pose
        if not self.place_on_cooling_tray(cycle, item_name, dimensions):
            return False
        # Close the oven door.
        return self.move_door(cycle, False)

    def run(self):
        # Run three complete bakery cycles.
        if not self.wait_until_ready():
            return False
        self.initialise_moveit()
        scene_ok = False
        for unused_attempt in range(10):
            try:
                scene_ok = self.moveit.get_planning_scene_monitor().request_planning_scene_state(
                    "/get_planning_scene"
                )
            except Exception:
                scene_ok = False
            if scene_ok:
                break
            time.sleep(1.0)
        if not scene_ok:
            self.get_logger().error("MoveIt did not return the planning scene")
            return False
        for cycle, item_name in enumerate(self.sequence, start=1):
            self.log_state(
                f"Starting cycle {cycle}/3: {self.items[item_name]['label']}"
            )
            if not self.run_cycle(cycle, item_name):
                self.get_logger().error(
                    f"Cycle {cycle} failed while handling {item_name}"
                )
                return False
        if not self.move_home():
            self.get_logger().warning("Task finished, but the home move failed")
        self.log_state("All three bakery cycles completed")
        return True

    def shutdown_moveit(self):
        # Stop the MoveIt interface.
        if self.moveit is not None:
            self.arm = None
            self.plan_parameters = None
            gc.collect()
            self.moveit.shutdown()
            self.moveit = None
            time.sleep(0.25)


def main(args=None):
    # Start the bakery task node.
    rclpy.init(args=args)
    node = BakeryTaskNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    success = False
    try:
        success = node.run()
        while success and rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        node.get_logger().warning("Bakery task interrupted")
    except Exception as error:
        node.get_logger().error(f"Bakery task failed: {error}")
    finally:
        node.shutdown_moveit()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
