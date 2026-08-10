import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_profiles(package_share):
    # Load the available finger profiles.
    path = os.path.join(package_share, "config", "finger_profiles.yaml")
    with open(path, encoding="utf-8") as profile_file:
        return yaml.safe_load(profile_file)["profiles"]


def launch_setup(context):
    # Create matching simulation and MoveIt descriptions.
    package_share = get_package_share_directory("auto_cake_feeder")
    finger_name = LaunchConfiguration("finger_profile").perform(context)
    bakery_item = LaunchConfiguration("bakery_item").perform(context)
    spawn_three_items = LaunchConfiguration("spawn_three_items").perform(context)
    object_sequence = LaunchConfiguration("object_sequence").perform(context)
    simulator_args = LaunchConfiguration("simulator_args").perform(context)
    profiles = load_profiles(package_share)
    if finger_name not in profiles:
        raise RuntimeError(f"Unknown finger profile '{finger_name}'. Choices: {sorted(profiles)}")

    finger = profiles[finger_name]
    mappings = {
        "finger_mesh": str(finger["mesh"]),
        "finger_mesh_origin_xyz": str(finger["mesh_origin_xyz"]),
        "finger_mesh_origin_rpy": str(finger["mesh_origin_rpy"]),
        "finger_collision_origin_xyz": str(finger["collision_origin_xyz"]),
        "finger_collision_size": str(finger["collision_size"]),
        "finger_mass": str(finger["finger_mass"]),
        "gripper_closed_position": str(finger["closed_position"]),
        "gripper_open_position": str(finger["open_position"]),
        "gripper_tcp_xyz": str(finger["tcp_xyz"]),
    }

    moveit_config = (
        MoveItConfigsBuilder(
            "auto_cake_feeder_ur5",
            package_name="auto_cake_feeder",
        )
        .robot_description(
            file_path="urdf/auto_cake_feeder.urdf.xacro",
            mappings=mappings,
        )
        .robot_description_semantic(file_path="config/auto_cake_feeder.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
            load_all=False,
        )
        .to_moveit_configs()
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "finger_profile": finger_name,
            "bakery_item": bakery_item,
            "spawn_three_items": spawn_three_items,
            "object_sequence": object_sequence,
            "simulator_args": simulator_args,
        }.items(),
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        ros_arguments=["--log-level", "warn"],
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "allow_trajectory_execution": True,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
        ],
    )
    planning_scene = Node(
        package="auto_cake_feeder",
        executable="planning_scene_node.py",
        output="screen",
        ros_arguments=["--log-level", "warn"],
        parameters=[{"use_sim_time": True}],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="screen",
        ros_arguments=["--log-level", "warn"],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        arguments=["-d", os.path.join(package_share, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )
    return [simulation, move_group, planning_scene, rviz]


def generate_launch_description():
    # Launch the bakery simulation with MoveIt and RViz.
    return LaunchDescription(
        [
            DeclareLaunchArgument("finger_profile", default_value="sample"),
            DeclareLaunchArgument("bakery_item", default_value="croissant"),
            DeclareLaunchArgument("spawn_three_items", default_value="false"),
            DeclareLaunchArgument(
                "object_sequence",
                default_value="croissant,cheese_roll,cupcake",
            ),
            DeclareLaunchArgument("simulator_args", default_value="empty.sdf"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
