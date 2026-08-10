cmake_minimum_required(VERSION 3.8)
project(auto_cake_feeder)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/bakery_task_node.py
    scripts/cell_state_monitor.py
    scripts/planning_scene_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
    models
    urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
    meshes/environment
    meshes/gripper
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()
