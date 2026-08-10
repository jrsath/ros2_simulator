#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

BLUE = "\033[1;94m"
RESET = "\033[0m"

class CarDriverNode(Node):
    # Set up the trigger, velocity publisher and timer.
    def __init__(self):
        super().__init__("car_driver_node")
        self.declare_parameter("forward_distance", 0.50)
        self.declare_parameter("forward_speed", 0.50)
        self.declare_parameter("publish_rate", 20.0)

        self.distance = float(self.get_parameter("forward_distance").value)
        self.speed = float(self.get_parameter("forward_speed").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        if self.distance <= 0.0 or self.speed <= 0.0 or publish_rate <= 0.0:
            raise ValueError(
                "forward_distance, forward_speed, and publish_rate must be positive"
            )

        self.drive_duration = self.distance / self.speed
        self.drive_timeout = max(3.0, self.drive_duration * 3.0)
        self.start_time = None
        self.start_xy = None
        self.current_xy = None
        self.finished = False
        self.velocity_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.trigger_subscription = self.create_subscription(
            Bool,
            "/trigger_car",
            self._trigger_callback,
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            "/model/two_wheel_car/odometry",
            self._odom_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_velocity)
        self.get_logger().info(
            f"{BLUE}Car driver ready; waiting for True on /trigger_car{RESET}"
        )

    # Start driving when a true trigger is received.
    def _trigger_callback(self, message):
        if not message.data or self.start_time is not None or self.finished:
            return
        self.start_time = self.get_clock().now()
        self.start_xy = self.current_xy
        self.get_logger().info(
            f"{BLUE}Car drive started: {self.distance:.2f} m forward at "
            f"{self.speed:.2f} m/s{RESET}"
        )

    # Store the latest car position from Gazebo.
    def _odom_callback(self, message):
        self.current_xy = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
        )
        if self.start_time is not None and self.start_xy is None:
            self.start_xy = self.current_xy

    # Publish forward velocity until the target distance is reached.
    def _publish_velocity(self):
        if self.start_time is None or self.finished:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1.0e9
        travelled = None
        if self.start_xy is not None and self.current_xy is not None:
            travelled = math.hypot(
                self.current_xy[0] - self.start_xy[0],
                self.current_xy[1] - self.start_xy[1],
            )
            if travelled >= self.distance:
                self.velocity_publisher.publish(Twist())
                self.finished = True
                self.get_logger().info(
                    f"{BLUE}Car drive complete: measured {travelled:.2f} m "
                    f"and stopped{RESET}"
                )
                return

        command = Twist()
        if elapsed < self.drive_timeout:
            command.linear.x = self.speed
            self.velocity_publisher.publish(command)
            return

        self.velocity_publisher.publish(command)
        self.finished = True
        distance_text = (
            "without odometry" if travelled is None else f"after {travelled:.2f} m"
        )
        self.get_logger().warning(
            f"Car drive timed out {distance_text}; the car was stopped"
        )

    # Publish a zero velocity command.
    def stop(self):
        self.velocity_publisher.publish(Twist())


# Run the car driver node.
def main(args=None):
    rclpy.init(args=args)
    node = CarDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
