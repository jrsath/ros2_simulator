from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os


package_path = get_package_share_directory("ur5_sim_package")

xacro_file = os.path.join(
    package_path,
    "urdf",
    "gripper_only.urdf.xacro",
)


# Build the gripper robot description.
def generate_robot_description():
    return ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
        ]),
        value_type=str,
    )


# Publish the gripper transforms.
def generate_robot_state_publisher():
    robot_description = generate_robot_description()

    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False,
        }],
    )


# Start the gripper joint-state publisher.
def generate_joint_state_publisher():
    robot_description = generate_robot_description()

    return Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False,
        }],
    )


# Start RViz for the gripper display.
def generate_rviz_node():
    return Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[{
            "use_sim_time": False,
        }],
    )


# Return the complete gripper display launch.
def generate_launch_description():
    return LaunchDescription([
        generate_robot_state_publisher(),
        generate_joint_state_publisher(),
        generate_rviz_node(),
    ])
