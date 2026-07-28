from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node 
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os

ur5_package = get_package_share_directory("ur5_sim_package")
ros_gz_sim_package = get_package_share_directory("ros_gz_sim")

xacro_file = os.path.join(
    ur5_package,
    "urdf",
    "ur.urdf.xacro",
)

gazebo_launch_file = os.path.join(
    ros_gz_sim_package,
    "launch",
    "gz_sim.launch.py",
)

car_sdf_file = os.path.join(
    ur5_package,
    "models",
    "two_wheel_car.sdf",
)

# Gazebo resolves package:// URIs as model:// URIs. Adding the parent of the
# package share directory lets it find model://ur5_sim_package/...
gazebo_resource_path = os.path.dirname(ur5_package)

def generate_clock_bridge():
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

def generate_robot_description():
    return ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " ",
            "ur_type:=ur5",
            " ",
            "sim_gazebo:=true",
            " ",
            "use_fake_hardware:=false",
        ]),
        value_type=str,
    )

def generate_robot_state_publisher():
    robot_description = generate_robot_description()
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }]   
    )

def generate_joint_state_publisher():
     robot_description = generate_robot_description()
     return Node(
         package='joint_state_publisher',
         executable='joint_state_publisher',
         output='screen',
         parameters=[{
             "robot_description": robot_description,
             "use_sim_time": True,
         }] 
     ) 

def generate_joint_state_broadcaster():
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )


def generate_arm_controller():
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )


def generate_gripper_controller():
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

def generate_rviz_node():
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        parameters=[{
            "use_sim_time": True,
        }] 
    )

def generate_gazebo_node():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            # OGRE2 crashes in this WSL setup while copying GPU textures.
            "gz_args": "-r empty.sdf",
        }.items(),
    )

def generate_spawn_ur5_node():
    return Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            "-world", "empty",
            '-topic', '/robot_description',
            '-name', 'ur5',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
    )

def generate_spawn_car_node():
    return Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_two_wheel_car",
        output="screen",
        arguments=[
            "-world", "empty",
            "-file", car_sdf_file,
            "-name", "two_wheel_car",
            "-x", "0.5",
            "-y", "0.5",
            "-z", "0.01",
        ],
    )

# Give Gazebo a few seconds to create the world before spawning.
def generate_spawn_node():
    return TimerAction(
        period=3.0,
        actions=[
            generate_spawn_ur5_node(),
            #generate_spawn_car_node(), 
        ],
    )

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch the basic RViz view from the simulation package.",
        ),
        generate_clock_bridge(),
        generate_robot_state_publisher(),

        generate_rviz_node(),

        generate_gazebo_node(),
        generate_spawn_node(),

        generate_joint_state_broadcaster(),
        #generate_joint_state_publisher(),
        generate_arm_controller(),
        generate_gripper_controller(),
        
    ])






