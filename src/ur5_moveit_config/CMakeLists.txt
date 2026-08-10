cmake_minimum_required(VERSION 3.8)
project(ur5_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/car_driver_node.py
    scripts/pick_and_place_node.py
    scripts/planning_scene_obstacles.py
    scripts/run_pick_and_place_quiet
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    .setup_assistant
    README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
