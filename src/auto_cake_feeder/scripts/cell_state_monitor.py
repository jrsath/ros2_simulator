#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CellStateMonitor(Node):
    # Report the physical oven door and button states.

    def __init__(self):
        # Create the oven joint-state subscriber.
        super().__init__("cell_state_monitor")
        self.door_open = None
        self.button_pressed = None
        self.create_subscription(
            JointState,
            "/oven/joint_state",
            self.on_joint_state,
            10,
        )

    def on_joint_state(self, message):
        # Log a state only when it changes.
        positions = dict(zip(message.name, message.position))
        door_angle = positions.get("oven_door_joint")
        button_position = positions.get("start_button_joint")

        if door_angle is not None:
            door_open = door_angle < -0.25
            if door_open != self.door_open:
                self.door_open = door_open
                state = "open" if door_open else "closed"
                self.get_logger().info(f"Oven door: {state}")

        if button_position is not None:
            button_pressed = button_position > 0.004
            if button_pressed != self.button_pressed:
                self.button_pressed = button_pressed
                state = "pressed" if button_pressed else "released"
                self.get_logger().info(f"Oven start button: {state}")


def main(args=None):
    # Run the cell state monitor.
    rclpy.init(args=args)
    node = CellStateMonitor()
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
