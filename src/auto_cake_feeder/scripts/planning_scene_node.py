#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


class PlanningSceneNode(Node):
    # Publish the fixed cell and moving oven door to MoveIt.

    def __init__(self):
        # Create the scene publisher and door subscriber.
        super().__init__("bakery_planning_scene")
        self.publisher = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/bakery_layout_markers", 10
        )
        self.door_angle = 0.0
        self.door_collision_enabled = True
        self.button_contact_enabled = False
        self.shelf_access_enabled = False
        self.shelf_area_removed = False
        self.door_removed = False
        self.create_subscription(JointState, "/oven/joint_state", self.on_oven_state, 10)
        self.create_subscription(
            Bool,
            "/door_collision_enabled",
            self.on_door_collision_enabled,
            10,
        )
        self.create_subscription(
            Bool,
            "/button_contact_enabled",
            self.on_button_contact_enabled,
            10,
        )
        self.create_subscription(
            Bool,
            "/shelf_access_enabled",
            self.on_shelf_access_enabled,
            10,
        )
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            0.25,
            self.publish_scene,
            clock=steady_clock,
        )
        print(
            "\033[1;94m[BAKERY LAYOUT] Loaded measured 2026-08-09 layout\033[0m",
            flush=True,
        )

    def box(self, object_id, xyz, size, yaw=0.0):
        # Create one box collision object.
        collision = CollisionObject()
        collision.header.frame_id = "world"
        collision.id = object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(size)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        collision.primitives.append(primitive)
        collision.primitive_poses.append(pose)
        collision.operation = CollisionObject.ADD
        return collision

    def on_oven_state(self, message):
        # Save the latest physical door angle.
        positions = dict(zip(message.name, message.position))
        if "oven_door_joint" in positions:
            self.door_angle = positions["oven_door_joint"]

    def on_door_collision_enabled(self, message):
        # Enable the door obstacle except while it is being handled.
        self.door_collision_enabled = message.data

    def on_button_contact_enabled(self, message):
        # Allow the tool to press the physical oven button.
        self.button_contact_enabled = message.data

    def on_shelf_access_enabled(self, message):
        # Allow the tool to enter the shelf while collecting a tray.
        self.shelf_access_enabled = message.data

    def fixed_objects(self):
        # Create the fixed collision objects in the cell.
        objects = [
            self.box("worktop", (0.0, -0.042, 0.790), (1.50, 1.50, 0.010)),
            self.box("frame_rear", (0.0, 0.683, 1.375), (1.30, 0.050, 1.16)),
            self.box("frame_left", (-0.625, -0.042, 1.375), (0.050, 1.40, 1.16)),
            self.box("frame_right", (0.625, -0.042, 1.375), (0.050, 1.40, 1.16)),
            self.box("shelf_left", (-0.407, -0.647, 1.085), (0.200, 0.010, 0.580)),
            self.box("shelf_right", (-0.407, -0.337, 1.085), (0.200, 0.010, 0.580)),
            self.box("shelf_back", (-0.502, -0.492, 1.085), (0.010, 0.320, 0.580)),
            self.box("shelf_bottom", (-0.407, -0.492, 0.797), (0.200, 0.320, 0.004)),
            self.box("cooling_tray", (-0.407, -0.492, 1.380), (0.180, 0.320, 0.003)),
            self.box("oven_bottom", (0.335, -0.4395, 0.810), (0.370, 0.465, 0.030)),
            self.box("oven_top", (0.335, -0.4395, 1.150), (0.370, 0.465, 0.030)),
            self.box("oven_left", (0.335, -0.217, 0.980), (0.370, 0.020, 0.370)),
            self.box("oven_right", (0.335, -0.662, 0.980), (0.370, 0.020, 0.370)),
            self.box("oven_back", (0.510, -0.4395, 0.980), (0.020, 0.425, 0.340)),
            self.box("oven_front_left", (0.155, -0.237, 1.0425), (0.010, 0.060, 0.175)),
            self.box("oven_front_right", (0.155, -0.642, 1.0425), (0.010, 0.060, 0.175)),
            self.box("oven_bed", (0.340, -0.4395, 0.875), (0.280, 0.300, 0.010)),
        ]
        if self.button_contact_enabled:
            # Keep the panel collision, with a small opening around the button.
            objects.extend([
                self.box(
                    "oven_front_panel_left",
                    (0.155, -0.631, 0.875),
                    (0.010, 0.042, 0.160),
                ),
                self.box(
                    "oven_front_panel_right",
                    (0.155, -0.3685, 0.875),
                    (0.010, 0.283, 0.160),
                ),
            ])
        else:
            objects.append(self.box(
                "oven_front_panel",
                (0.155, -0.4395, 0.875),
                (0.010, 0.425, 0.160),
            ))
        ignored = set()
        if self.shelf_access_enabled:
            ignored.update({
                "worktop",
                "shelf_left",
                "shelf_right",
                "shelf_back",
                "shelf_bottom",
            })
        return [item for item in objects if item.id not in ignored]

    def door_object(self):
        # Create the door at its measured joint angle.
        hinge_x, hinge_y, hinge_z = 0.145, -0.267, 1.0425
        world_yaw = -math.pi / 2.0 + self.door_angle
        centre_x = hinge_x + 0.1725 * math.cos(world_yaw)
        centre_y = hinge_y + 0.1725 * math.sin(world_yaw)
        return self.box(
            "oven_door",
            (centre_x, centre_y, hinge_z),
            (0.345, 0.008, 0.175),
            world_yaw,
        )

    def publish_layout_markers(self):
        # Show planned tray levels without adding collision objects.
        markers = MarkerArray()

        enclosure = Marker()
        enclosure.header.frame_id = "world"
        enclosure.ns = "cell_visuals"
        enclosure.id = 1
        enclosure.type = Marker.MESH_RESOURCE
        enclosure.action = Marker.ADD
        enclosure.pose.orientation.w = 1.0
        enclosure.scale.x = 0.001
        enclosure.scale.y = 0.001
        enclosure.scale.z = 0.001
        enclosure.color.r = 0.92
        enclosure.color.g = 0.90
        enclosure.color.b = 0.84
        enclosure.color.a = 1.0
        enclosure.mesh_resource = (
            "package://auto_cake_feeder/meshes/environment/cell_frame.STL"
        )
        enclosure.mesh_use_embedded_materials = False
        markers.markers.append(enclosure)

        for marker_id in (10, 11, 12, 13):
            old_box = Marker()
            old_box.header.frame_id = "world"
            old_box.ns = "cell_visuals"
            old_box.id = marker_id
            old_box.action = Marker.DELETE
            markers.markers.append(old_box)

        measurements = (
            (30, (-0.550, -0.792), (-0.550, -0.652), "Shelf front: 140 mm"),
            (31, (-0.600, -0.492), (-0.507, -0.492), "Shelf side: 93 mm"),
            (32, (0.550, -0.792), (0.550, -0.672), "Oven front: 120 mm"),
            (33, (0.520, -0.4395), (0.600, -0.4395), "Oven side: 80 mm"),
        )
        for marker_id, start, end, text in measurements:
            line = Marker()
            line.header.frame_id = "world"
            line.ns = "measured_offsets"
            line.id = marker_id
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.006
            line.color.r = 0.10
            line.color.g = 0.75
            line.color.b = 1.0
            line.color.a = 1.0
            line.points = [
                Point(x=start[0], y=start[1], z=0.815),
                Point(x=end[0], y=end[1], z=0.815),
            ]
            markers.markers.append(line)

            label = Marker()
            label.header.frame_id = "world"
            label.ns = "measured_offsets"
            label.id = marker_id + 10
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = (start[0] + end[0]) / 2.0
            label.pose.position.y = (start[1] + end[1]) / 2.0
            label.pose.position.z = 0.845
            label.pose.orientation.w = 1.0
            label.scale.z = 0.025
            label.color.r = 0.10
            label.color.g = 0.75
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = text
            markers.markers.append(label)

        for index, height in enumerate((0.900, 1.000, 1.100), start=1):
            level = Marker()
            level.header.frame_id = "world"
            level.ns = "oven_tray_targets"
            level.id = index
            level.type = Marker.CUBE
            level.action = Marker.ADD
            level.pose.position.x = 0.340
            level.pose.position.y = -0.4395
            level.pose.position.z = height
            level.pose.orientation.w = 1.0
            level.scale.x = 0.280
            level.scale.y = 0.300
            level.scale.z = 0.002
            level.color.r = 1.0
            level.color.g = 0.65
            level.color.b = 0.05
            level.color.a = 0.28
            markers.markers.append(level)

            label = Marker()
            label.header.frame_id = "world"
            label.ns = "oven_tray_target_labels"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = 0.0675
            label.pose.position.y = -0.4395
            label.pose.position.z = height + 0.025
            label.pose.orientation.w = 1.0
            label.scale.z = 0.032
            label.color.r = 0.20
            label.color.g = 0.70
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"Tray target {index}: z={height:.3f} m"
            markers.markers.append(label)
        self.marker_publisher.publish(markers)

    def publish_scene(self):
        # Publish a complete scene update.
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = self.fixed_objects()
        if self.button_contact_enabled:
            remove = CollisionObject()
            remove.header.frame_id = "world"
            remove.id = "oven_front_panel"
            remove.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(remove)
        else:
            for object_id in (
                "oven_front_panel_left",
                "oven_front_panel_right",
            ):
                remove = CollisionObject()
                remove.header.frame_id = "world"
                remove.id = object_id
                remove.operation = CollisionObject.REMOVE
                scene.world.collision_objects.append(remove)
        if self.shelf_access_enabled:
            if not self.shelf_area_removed:
                for object_id in (
                    "worktop",
                    "shelf_left",
                    "shelf_right",
                    "shelf_back",
                    "shelf_bottom",
                ):
                    remove = CollisionObject()
                    remove.header.frame_id = "world"
                    remove.id = object_id
                    remove.operation = CollisionObject.REMOVE
                    scene.world.collision_objects.append(remove)
                self.shelf_area_removed = True
        else:
            self.shelf_area_removed = False
        if self.door_collision_enabled:
            scene.world.collision_objects.append(self.door_object())
            self.door_removed = False
        else:
            if not self.door_removed:
                remove_door = CollisionObject()
                remove_door.header.frame_id = "world"
                remove_door.id = "oven_door"
                remove_door.operation = CollisionObject.REMOVE
                scene.world.collision_objects.append(remove_door)
                self.door_removed = True
        self.publisher.publish(scene)
        self.publish_layout_markers()


def main(args=None):
    # Run the planning scene node.
    rclpy.init(args=args)
    node = PlanningSceneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
