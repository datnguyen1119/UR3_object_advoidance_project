cmake_minimum_required(VERSION 3.8)
project(custom_ur_simulation)

find_package(ament_cmake REQUIRED)

# This package now only ships data + launch files. The C++ nodes that used
# to live here moved to custom_ur_servo and custom_ur_teleop.

install(
  DIRECTORY
    launch
    config
    worlds
    rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
