"""Launch Gazebo, MoveIt/RViz, then the assignment pick-and-place node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


# Return the complete autonomous task launch.
def generate_launch_description():
    package_share = get_package_share_directory("ur5_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder(
            "group4_ur5",
            package_name="ur5_moveit_config",
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )

    launch_rviz = LaunchConfiguration("launch_rviz")
    simulator_args = LaunchConfiguration("simulator_args")
    startup_delay = LaunchConfiguration("startup_delay")
    run_pipeline = LaunchConfiguration("run_pipeline")

    base_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "moveit.launch.py")
        ),
        launch_arguments={
            "launch_simulator": "true",
            "launch_rviz": launch_rviz,
            "simulator_args": simulator_args,
        }.items(),
    )

    # MoveItPy planning-scene settings.
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

    pick_and_place = Node(
        package="ur5_moveit_config",
        executable="pick_and_place_node.py",
        name="pick_and_place_node",
        output="both",
        emulate_tty=True,
        ros_arguments=[
            "--log-level",
            "warn",
            "--log-level",
            "pick_and_place_node:=info",
        ],
        condition=IfCondition(run_pipeline),
        parameters=[
            moveit_config.to_dict(),
            moveit_py_options,
            {
                # Car outer bounds.
                "car_x": 0.224,
                "car_y": 0.0,
                "car_z": 0.02,
                "car_length": 0.10,
                "car_width": 0.07,
                "car_height": 0.04,
                # Pedestal pose.
                "pedestal_x": 0.50,
                "pedestal_y": 0.0,
                "pedestal_top": 0.80,
            },
        ],
    )

    car_driver = Node(
        package="ur5_moveit_config",
        executable="car_driver_node.py",
        name="car_driver_node",
        output="screen",
        ros_arguments=[
            "--log-level",
            "warn",
            "--log-level",
            "car_driver_node:=info",
        ],
        condition=IfCondition(run_pipeline),
        parameters=[{
            "use_sim_time": True,
            "forward_distance": 0.50,
            "forward_speed": 0.50,
            "publish_rate": 20.0,
        }],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz with the MoveIt planning display.",
            ),
            DeclareLaunchArgument(
                "simulator_args",
                default_value="empty.sdf",
                description=(
                    "Gazebo arguments; use '-s empty.sdf' for server-only mode."
                ),
            ),
            DeclareLaunchArgument(
                "startup_delay",
                default_value="8.0",
                description=(
                    "Seconds allowed for Gazebo, controllers, and move_group to start."
                ),
            ),
            DeclareLaunchArgument(
                "run_pipeline",
                default_value="true",
                description="Run the pick-and-place state machine after startup.",
            ),
            base_system,
            car_driver,
            TimerAction(period=startup_delay, actions=[pick_and_place]),
        ]
    )
