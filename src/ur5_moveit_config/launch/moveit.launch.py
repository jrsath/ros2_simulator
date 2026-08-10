from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import os


# Return the Gazebo, MoveIt and RViz launch.
def generate_launch_description():
    moveit_package = get_package_share_directory("ur5_moveit_config")
    simulation_package = get_package_share_directory("ur5_sim_package")

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

    launch_simulator = LaunchConfiguration("launch_simulator")
    launch_rviz = LaunchConfiguration("launch_rviz")
    simulator_args = LaunchConfiguration("simulator_args")
    scene_file = os.path.join(
        moveit_package, "config", "planning_scene_objects.yaml"
    )

    # Scope the included launch arguments so launch_rviz:=false only disables
    # the simulator package's basic RViz, not the MoveIt RViz below.
    simulator = GroupAction(
        condition=IfCondition(launch_simulator),
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(simulation_package, "launch", "ur5.launch.py")
                ),
                launch_arguments={
                    "launch_rviz": "false",
                    "simulator_args": simulator_args,
                }.items(),
            ),
        ],
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
                "monitor_dynamics": False,
            },
        ],
    )

    planning_scene_obstacles = Node(
        package="ur5_moveit_config",
        executable="planning_scene_obstacles.py",
        name="planning_scene_obstacles",
        output="screen",
        ros_arguments=[
            "--log-level",
            "warn",
            "--log-level",
            "planning_scene_obstacles:=info",
        ],
        parameters=[
            {
                "scene_file": scene_file,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="screen",
        ros_arguments=["--log-level", "warn"],
        condition=IfCondition(launch_rviz),
        arguments=[
            "-d",
            os.path.join(moveit_package, "config", "moveit.rviz"),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_simulator",
            default_value="true",
            description="Launch Gazebo, ros2_control, and the robot description.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz with the MoveIt MotionPlanning panel.",
        ),
        DeclareLaunchArgument(
            "simulator_args",
            default_value="empty.sdf",
            description=(
                "Gazebo arguments. Use '-s empty.sdf' for server-only mode."
            ),
        ),
        simulator,
        move_group,
        planning_scene_obstacles,
        rviz,
    ])
