import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_profiles(package_share):
    # Load the finger profile file.
    path = os.path.join(package_share, "config", "finger_profiles.yaml")
    with open(path, encoding="utf-8") as profile_file:
        return yaml.safe_load(profile_file)["profiles"]


def launch_setup(context):
    # Create the simulation and matching task node.
    package_share = get_package_share_directory("auto_cake_feeder")
    finger_name = LaunchConfiguration("finger_profile").perform(context)
    object_sequence = LaunchConfiguration("object_sequence").perform(context)
    startup_delay = float(LaunchConfiguration("startup_delay").perform(context))
    automatic_bake_delay = float(
        LaunchConfiguration("automatic_bake_delay").perform(context)
    )
    velocity_scaling = float(LaunchConfiguration("velocity_scaling").perform(context))
    acceleration_scaling = float(
        LaunchConfiguration("acceleration_scaling").perform(context)
    )
    profiles = load_profiles(package_share)
    if finger_name not in profiles:
        raise RuntimeError(
            f"Unknown finger profile '{finger_name}'. Choices: {sorted(profiles)}"
        )

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

    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "moveit.launch.py")
        ),
        launch_arguments={
            "finger_profile": finger_name,
            "spawn_three_items": "true",
            "object_sequence": object_sequence,
            "simulator_args": LaunchConfiguration("simulator_args"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
        }.items(),
    )
    moveit_py_options = {
        "use_sim_time": True,
        "planning_scene_monitor_options.name": "planning_scene_monitor",
        "planning_scene_monitor_options.robot_description": "robot_description",
        "planning_scene_monitor_options.joint_state_topic": "/joint_states",
        "planning_scene_monitor_options.attached_collision_object_topic": (
            "/moveit_cpp/planning_scene_monitor"
        ),
        "planning_scene_monitor_options.publish_planning_scene_topic": (
            "/moveit_cpp/publish_planning_scene"
        ),
        "planning_scene_monitor_options.monitored_planning_scene_topic": (
            "/monitored_planning_scene"
        ),
        "planning_scene_monitor_options.wait_for_initial_state_timeout": 15.0,
        "planning_pipelines.pipeline_names": ["ompl"],
        "planning_pipelines.namespace": "",
    }
    task = Node(
        package="auto_cake_feeder",
        executable="bakery_task_node.py",
        name="bakery_task_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("run_task")),
        ros_arguments=[
            "--log-level",
            "warn",
            "--log-level",
            "bakery_task_node:=info",
        ],
        parameters=[
            moveit_config.to_dict(),
            moveit_py_options,
            {
                "object_sequence": object_sequence,
                "automatic_bake_delay": automatic_bake_delay,
                "velocity_scaling": velocity_scaling,
                "acceleration_scaling": acceleration_scaling,
            },
        ],
    )
    return [system, TimerAction(period=startup_delay, actions=[task])]


def generate_launch_description():
    # Launch the complete three-item bakery task.
    return LaunchDescription(
        [
            DeclareLaunchArgument("finger_profile", default_value="sample"),
            DeclareLaunchArgument(
                "object_sequence",
                default_value="croissant,cheese_roll,cupcake",
            ),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("simulator_args", default_value="empty.sdf"),
            DeclareLaunchArgument("startup_delay", default_value="12.0"),
            DeclareLaunchArgument("automatic_bake_delay", default_value="0.0"),
            DeclareLaunchArgument("velocity_scaling", default_value="0.30"),
            DeclareLaunchArgument("acceleration_scaling", default_value="0.22"),
            DeclareLaunchArgument("run_task", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
