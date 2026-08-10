#!/usr/bin/env python3
import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


# Convert roll, pitch and yaw to a quaternion.
def quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class PlanningSceneObstacles(Node):
    # Load the scene file and prepare the MoveIt service.
    def __init__(self):
        super().__init__("planning_scene_obstacles")
        self.declare_parameter("scene_file", "")
        scene_file = Path(
            self.get_parameter("scene_file").get_parameter_value().string_value
        )
        if not scene_file.is_file():
            raise RuntimeError(f"Planning-scene file does not exist: {scene_file}")

        with scene_file.open("r", encoding="utf-8") as stream:
            scene = yaml.safe_load(stream) or {}

        self._frame_id = str(scene.get("frame_id", "world"))
        self._objects = [
            self._make_object(item)
            for item in scene.get("objects", [])
            if item.get("enabled", True)
        ]

        self._client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self._future = None
        self._timer = self.create_timer(0.5, self._apply_scene)

    # Build one collision object from its settings.
    def _make_object(self, item):
        object_id = str(item["id"])
        parts = item.get("parts", [item])

        collision_object = CollisionObject()
        collision_object.header.frame_id = self._frame_id
        collision_object.id = object_id
        collision_object.operation = CollisionObject.ADD

        for part in parts:
            primitive, pose = self._make_primitive(part, object_id)
            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(pose)
        return collision_object

    # Build one collision shape and pose.
    @staticmethod
    def _make_primitive(item, object_id):
        shape_name = str(item.get("shape", "box")).lower()
        dimensions = [float(value) for value in item["dimensions"]]
        pose_values = [float(value) for value in item["pose"]]
        if len(pose_values) != 6:
            raise ValueError(
                f"{object_id}: pose must be [x, y, z, roll, pitch, yaw]"
            )

        primitive = SolidPrimitive()
        expected_dimensions = {"box": 3, "sphere": 1, "cylinder": 2}
        primitive_types = {
            "box": SolidPrimitive.BOX,
            "sphere": SolidPrimitive.SPHERE,
            "cylinder": SolidPrimitive.CYLINDER,
        }
        if shape_name not in primitive_types:
            raise ValueError(f"{object_id}: unsupported shape '{shape_name}'")
        if len(dimensions) != expected_dimensions[shape_name]:
            raise ValueError(
                f"{object_id}: {shape_name} requires "
                f"{expected_dimensions[shape_name]} dimensions"
            )
        primitive.type = primitive_types[shape_name]
        primitive.dimensions = dimensions

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pose_values[:3]
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = quaternion_from_rpy(*pose_values[3:])
        return primitive, pose

    # Send the collision objects to MoveIt.
    def _apply_scene(self):
        if not self._objects:
            self.get_logger().warn("No enabled planning-scene objects were configured")
            self._timer.cancel()
            return

        if self._future is not None or not self._client.service_is_ready():
            return

        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.world.collision_objects = self._objects
        self._future = self._client.call_async(request)
        self._future.add_done_callback(self._scene_applied)

    # Check whether MoveIt accepted the scene.
    def _scene_applied(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warn(f"Could not apply planning scene: {error}")
            self._future = None
            return

        if response.success:
            names = ", ".join(item.id for item in self._objects)
            self.get_logger().info(f"Applied MoveIt collision objects: {names}")
            self._timer.cancel()
        else:
            self.get_logger().warn("MoveIt rejected the planning-scene update; retrying")
            self._future = None


# Run the planning-scene node.
def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
