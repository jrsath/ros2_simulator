from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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

assignment_environment_sdf_file = os.path.join(
    ur5_package,
    "models",
    "assignment_environment.sdf",
)

# Gazebo resource path.
gazebo_resource_path = os.path.dirname(ur5_package)
quiet_ros_arguments = ["--log-level", "warn"]

# Bridge the Gazebo clock to ROS.
def generate_clock_bridge():
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        ros_arguments=quiet_ros_arguments,
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # Car velocity bridge.
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            # Car odometry bridge.
            "/model/two_wheel_car/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            # Dynamic model poses for physical pick verification.
            "/world/empty/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
    )

# Build the complete robot description.
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

# Publish the robot transforms.
def generate_robot_state_publisher():
    robot_description = generate_robot_description()
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        ros_arguments=quiet_ros_arguments,
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }]
    )

# Load the joint-state publisher settings.
def generate_joint_state_publisher():
     robot_description = generate_robot_description()
     return Node(
         package='joint_state_publisher',
         executable='joint_state_publisher',
         output='screen',
         ros_arguments=quiet_ros_arguments,
         parameters=[{
             "robot_description": robot_description,
             "use_sim_time": True,
         }]
     )

# Start all robot controllers together.
def generate_controller_spawner():
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "arm_controller",
            "gripper_controller",
            "--activate-as-group",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--switch-timeout",
            "60",
        ],
        output="screen",
        ros_arguments=quiet_ros_arguments,
    )

# Start RViz when requested.
def generate_rviz_node():
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        ros_arguments=quiet_ros_arguments,
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        parameters=[{
            "use_sim_time": True,
        }]
    )

# Start Gazebo Fortress.
def generate_gazebo_node():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            # OGRE2 crashes in this WSL setup while copying GPU textures.
            "gz_args": LaunchConfiguration("simulator_args"),
        }.items(),
    )

# Spawn the UR5 and gripper in Gazebo.
def generate_spawn_ur5_node():
    return Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        ros_arguments=quiet_ros_arguments,
        arguments=[
            "-world", "empty",
            '-topic', '/robot_description',
            '-name', 'ur5',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0",
        ],
    )

# Spawn the car in Gazebo.
def generate_spawn_car_node():
    return Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_two_wheel_car",
        output="screen",
        ros_arguments=quiet_ros_arguments,
        arguments=[
            "-world", "empty",
            "-file", car_sdf_file,
            "-name", "two_wheel_car",
            "-x", "0.2",
            "-y", "0.0",
            "-z", "0.05",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0",

        ],
    )

# Spawn the ground and pedestal.
def generate_spawn_environment_node():
    return Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_assignment_environment",
        output="screen",
        ros_arguments=quiet_ros_arguments,
        arguments=[
            "-world", "empty",
            "-file", assignment_environment_sdf_file,
            "-name", "assignment_environment",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
    )


# Start the Gazebo physics simulation.
def generate_start_simulation():
    return ExecuteProcess(
        cmd=[
            "ign", "service",
            "-s", "/world/empty/control",
            "--reqtype", "ignition.msgs.WorldControl",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "5000",
            "--req", "pause: false",
        ],
        output="screen",
    )


# Advance enough paused steps for ros2_control to activate.
def generate_controller_startup_steps(step_count):
    return ExecuteProcess(
        cmd=[
            "ign", "service",
            "-s", "/world/empty/control",
            "--reqtype", "ignition.msgs.WorldControl",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "5000",
            "--req", f"pause: true, multi_step: {step_count}",
        ],
        output="screen",
    )

# Give Gazebo a few seconds to create the world before spawning.
# Group the model-spawning actions.
def generate_spawn_node(spawn_ur5):
    return TimerAction(
        period=3.0,
        actions=[
            generate_spawn_environment_node(),
            generate_spawn_car_node(),
            TimerAction(period=1.0, actions=[spawn_ur5]),
        ],
    )

# Return the complete simulation launch.
def generate_launch_description():
    spawn_ur5 = generate_spawn_ur5_node()
    controller_spawner = generate_controller_spawner()
    start_controller_setup = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_ur5,
            on_exit=[
                TimerAction(
                    period=0.5,
                    actions=[generate_controller_startup_steps(1)],
                ),
                TimerAction(
                    period=3.0,
                    actions=[generate_controller_startup_steps(100)],
                ),
            ],
        )
    )
    start_after_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_spawner,
            on_exit=[generate_start_simulation()],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch the basic RViz view from the simulation package.",
        ),
        DeclareLaunchArgument(
            "simulator_args",
            default_value="empty.sdf",
            description=(
                "Gazebo arguments. The launch resumes the initially paused world."
            ),
        ),
        AppendEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            gazebo_resource_path,
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            gazebo_resource_path,
        ),
        generate_clock_bridge(),
        generate_robot_state_publisher(),

        generate_rviz_node(),

        generate_gazebo_node(),
        generate_spawn_node(spawn_ur5),
        start_controller_setup,

        controller_spawner,
        start_after_controllers,

    ])
