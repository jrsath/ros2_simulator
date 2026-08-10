# Group 4 UR5 MoveIt 2 configuration

This package uses the custom robot description from `ur5_sim_package`.
`ur_control.xacro` remains the source of the Gazebo ROS 2 control interfaces.

## Assignment mapping

- Arm planning group: `ur_manipulator`
- Kinematic chain: `base_link` to `gripper_tcp`
- End effector group: `gripper`
- IK solver: KDL
- Planning pipeline: OMPL
- Arm execution controller: `arm_controller/follow_joint_trajectory`
- Gripper execution controller: `gripper_controller/gripper_cmd`

## Launch

```bash
ros2 launch ur5_moveit_config moveit.launch.py
```

Select `ur_manipulator` in the MotionPlanning panel. The interactive marker is
attached to `gripper_tcp`.

To reopen this package in MoveIt Setup Assistant, first install the missing
Humble GUI package:

```bash
sudo apt install ros-humble-moveit-setup-assistant
ros2 launch ur5_moveit_config setup_assistant.launch.py
```
