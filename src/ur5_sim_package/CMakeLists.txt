cmake_minimum_required(VERSION 3.8)
project(ur5_sim_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/measure_arm_tracking.py
    scripts/measure_gripper_tracking.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    urdf
    config
    meshes
    models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
