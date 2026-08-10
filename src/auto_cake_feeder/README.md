# Autonomous Cake Feeder

## Overview

This package simulates a UR5 bakery cell with:

- a UR5 arm on a 76 x 76 x 1000 mm support beam
- a Dynamixel parallel gripper with replaceable fingers
- an oven with a moving door and start button
- a fresh-tray shelf and cooling tray
- MoveIt 2 planning and collision checking

The task presses the oven button, collects a tray, inserts it into the oven,
waits for baking, opens the door, removes the item, places it on the cooling
tray and closes the door. It repeats the sequence for three bakery items.

## Main files

- `launch/task.launch.py` starts the complete task.
- `launch/moveit.launch.py` starts Gazebo, MoveIt and RViz without the task.
- `launch/simulation.launch.py` starts Gazebo and the controllers only.
- `scripts/bakery_task_node.py` contains the task and arm waypoints.
- `scripts/planning_scene_node.py` contains the MoveIt obstacles.
- `config/finger_profiles.yaml` contains the replaceable finger settings.
- `config/cell_layout.yaml` records the measured cell positions.

## Build

Open a terminal:

```bash
cd ~/group4_ros2_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build \
  --packages-select auto_cake_feeder \
  --symlink-install

source install/setup.bash
```

Run the build and source commands again after changing a mesh, URDF, launch file
or configuration file.

## Run the complete task

For an automatic demonstration:

```bash
cd ~/group4_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch auto_cake_feeder task.launch.py \
  automatic_bake_delay:=5.0
```

This starts Gazebo, MoveIt, RViz, the controllers and the bakery task.

For manual bake completion, run:

```bash
ros2 launch auto_cake_feeder task.launch.py
```

When State 4 appears, use another sourced terminal:

```bash
ros2 service call /bake_complete std_srvs/srv/Trigger "{}"
```

Send the service call once during each of the three cycles.

## Run without the automatic task

Gazebo, MoveIt and RViz:

```bash
ros2 launch auto_cake_feeder moveit.launch.py
```

Gazebo and controllers only:

```bash
ros2 launch auto_cake_feeder simulation.launch.py
```

## Change the arm path

Edit the `TASK WAYPOINTS - EDIT ROBOT POSITIONS HERE` block near the top of
`scripts/bakery_task_node.py`.

- `xyz` values are metres.
- `rpy` values are radians.
- `approach` positions stop before an object.
- `press`, `edge`, `inside`, `pickup` and `place` positions perform the task.

Change the task speed from the launch command:

```bash
ros2 launch auto_cake_feeder task.launch.py \
  automatic_bake_delay:=5.0 \
  velocity_scaling:=0.40 \
  acceleration_scaling:=0.30
```

The normal defaults are `0.30` velocity and `0.22` acceleration.

## Change the fingers

1. Put the new STL in `meshes/gripper/`.
2. Copy the `custom` entry in `config/finger_profiles.yaml`.
3. Set the mesh path, origin, collision box, mass, joint limits and TCP.
4. Build the package.
5. Launch using the profile name:

```bash
ros2 launch auto_cake_feeder task.launch.py \
  finger_profile:=custom \
  run_task:=false
```

The same profile is mirrored for the right finger automatically.

## Useful settings

- Cell measurements: `config/cell_layout.yaml`
- Bakery item sizes and masses: `config/bakery_items.yaml`
- Controller settings: `config/controllers.yaml`
- Arm waypoints: `scripts/bakery_task_node.py`
- MoveIt obstacles: `scripts/planning_scene_node.py`
