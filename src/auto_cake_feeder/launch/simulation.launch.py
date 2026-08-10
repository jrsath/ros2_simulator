import math
import os

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(path):
    # Load a YAML configuration file.
    with open(path, encoding="utf-8") as config_file:
        return yaml.safe_load(config_file)


def inertia_for_box(mass, dimensions):
    # Calculate inertia for a solid box.
    x, y, z = dimensions
    return (
        mass * (y * y + z * z) / 12.0,
        mass * (x * x + z * z) / 12.0,
        mass * (x * x + y * y) / 12.0,
    )


def inertia_for_cylinder(mass, radius, height):
    # Calculate inertia for a solid vertical cylinder.
    side = mass * (3.0 * radius * radius + height * height) / 12.0
    return side, side, mass * radius * radius / 2.0


def bakery_item_sdf(name, item):
    # Create one bakery item model from the size catalog.
    x, y, z = item["dimensions_m"]
    mass = float(item["mass_kg"])
    colour = " ".join(str(value) for value in item["colour"])

    if item["shape"] == "cylinder":
        radius = max(x, y) / 2.0
        geometry = f"<cylinder><radius>{radius}</radius><length>{z}</length></cylinder>"
        ixx, iyy, izz = inertia_for_cylinder(mass, radius, z)
    else:
        geometry = f"<box><size>{x} {y} {z}</size></box>"
        ixx, iyy, izz = inertia_for_box(mass, (x, y, z))

    return f"""<?xml version='1.0'?>
<sdf version='1.9'>
  <model name='{name}'>
    <link name='item'>
      <inertial>
        <mass>{mass}</mass>
        <inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name='collision'>
        <geometry>{geometry}</geometry>
        <surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface>
      </collision>
      <visual name='visual'>
        <geometry>{geometry}</geometry>
        <material><diffuse>{colour}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def tray_sdf(name):
    # Create a thin movable bakery tray.
    mass = 0.18
    x, y, z = 0.320, 0.180, 0.003
    ixx, iyy, izz = inertia_for_box(mass, (x, y, z))
    return f"""<?xml version='1.0'?>
<sdf version='1.9'>
  <model name='{name}'>
    <link name='tray'>
      <inertial><mass>{mass}</mass><inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name='collision'><geometry><box><size>{x} {y} {z}</size></box></geometry><surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>
      <visual name='visual'><geometry><box><size>{x} {y} {z}</size></box></geometry><material><diffuse>0.62 0.64 0.67 1</diffuse></material></visual>
    </link>
  </model>
</sdf>"""


def spawn_string(name, sdf, xyz, yaw=0.0):
    # Create a Gazebo model from an SDF string.
    return Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{name}",
        output="screen",
        ros_arguments=["--log-level", "warn"],
        arguments=[
            "-world", "empty",
            "-string", sdf,
            "-name", name,
            "-x", str(xyz[0]),
            "-y", str(xyz[1]),
            "-z", str(xyz[2]),
            "-Y", str(yaw),
        ],
    )


def startup_steps(step_count):
    # Advance the paused simulation during controller startup.
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


def start_simulation():
    # Unpause Gazebo after the controllers are active.
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


def launch_setup(context):
    # Load the selected finger and bakery item.
    package_share = get_package_share_directory("auto_cake_feeder")
    finger_name = LaunchConfiguration("finger_profile").perform(context)
    item_name = LaunchConfiguration("bakery_item").perform(context)
    spawn_three = LaunchConfiguration("spawn_three_items").perform(context).lower() == "true"
    object_sequence = [
        name.strip()
        for name in LaunchConfiguration("object_sequence").perform(context).split(",")
        if name.strip()
    ]

    profiles = load_yaml(os.path.join(package_share, "config", "finger_profiles.yaml"))["profiles"]
    items = load_yaml(os.path.join(package_share, "config", "bakery_items.yaml"))["items"]
    if finger_name not in profiles:
        raise RuntimeError(f"Unknown finger profile '{finger_name}'. Choices: {sorted(profiles)}")
    if item_name not in items:
        raise RuntimeError(f"Unknown bakery item '{item_name}'. Choices: {sorted(items)}")
    unknown_items = set(object_sequence) - set(items)
    if unknown_items:
        raise RuntimeError(f"Unknown bakery items: {sorted(unknown_items)}")
    if spawn_three and len(object_sequence) != 3:
        raise RuntimeError("object_sequence must contain exactly three items")
    if spawn_three and len(set(object_sequence)) != 3:
        raise RuntimeError("object_sequence items must have unique names")

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
    robot_description = xacro.process_file(
        os.path.join(package_share, "urdf", "auto_cake_feeder.urdf.xacro"),
        mappings=mappings,
    ).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        ros_arguments=["--log-level", "warn"],
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_auto_cake_feeder_ur5",
        output="screen",
        ros_arguments=["--log-level", "warn"],
        arguments=["-world", "empty", "-topic", "/robot_description", "-name", "ur5"],
    )
    spawn_environment = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_cell_environment",
        output="screen",
        ros_arguments=["--log-level", "warn"],
        arguments=[
            "-world", "empty",
            "-file", os.path.join(package_share, "models", "cell_environment", "model.sdf"),
            "-name", "cell_environment",
        ],
    )
    spawn_oven = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_oven",
        output="screen",
        ros_arguments=["--log-level", "warn"],
        arguments=[
            "-world", "empty",
            "-file", os.path.join(package_share, "models", "oven", "model.sdf"),
            "-name", "oven",
            "-x", "0.335", "-y", "-0.4395", "-z", "0.795",
            "-Y", str(-math.pi / 2.0),
        ],
    )

    selected_items = object_sequence if spawn_three else [item_name]
    item_offsets = [-0.090, 0.0, 0.090] if spawn_three else [0.0]
    item_spawns = []
    for selected_name, item_offset in zip(selected_items, item_offsets):
        item_height = float(items[selected_name]["dimensions_m"][2])
        item_spawns.append(
            spawn_string(
                selected_name,
                bakery_item_sdf(selected_name, items[selected_name]),
                (0.340, -0.4395 - item_offset, 0.880 + item_height / 2.0),
                -math.pi / 2.0,
            )
        )
    trays = [
        spawn_string(
            f"fresh_tray_{index + 1}",
            tray_sdf(f"fresh_tray_{index + 1}"),
            (-0.407, -0.492, 0.8005 + index * 0.0032),
            math.pi / 2.0,
        )
        for index in range(3)
    ]

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        ros_arguments=["--log-level", "warn"],
        arguments=[
            "joint_state_broadcaster",
            "arm_controller",
            "gripper_controller",
            "--activate-as-group",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "60",
        ],
    )

    controller_steps = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(period=0.5, actions=[startup_steps(1)]),
                TimerAction(period=2.0, actions=[startup_steps(100)]),
                TimerAction(period=4.0, actions=[start_simulation()]),
                TimerAction(period=4.1, actions=[controller_spawner]),
            ],
        )
    )

    return [
        robot_state_publisher,
        TimerAction(
            period=3.0,
            actions=[spawn_environment, spawn_oven, *trays, *item_spawns, spawn_robot],
        ),
        controller_steps,
    ]


def generate_launch_description():
    # Launch Gazebo, the measured cell, UR5 and selected fingers.
    package_share = get_package_share_directory("auto_cake_feeder")
    ros_gz_share = get_package_share_directory("ros_gz_sim")
    resource_root = os.path.dirname(package_share)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": LaunchConfiguration("simulator_args")}.items(),
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bakery_cell_bridge",
        output="screen",
        ros_arguments=["--log-level", "warn"],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/oven/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/world/empty/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
    )
    monitor = Node(
        package="auto_cake_feeder",
        executable="cell_state_monitor.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

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
            AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_root),
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_root),
            bridge,
            gazebo,
            OpaqueFunction(function=launch_setup),
            monitor,
        ]
    )
