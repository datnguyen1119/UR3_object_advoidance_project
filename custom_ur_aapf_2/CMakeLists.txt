cmake_minimum_required(VERSION 3.8)
project(custom_ur_aapf_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# ----------------------------------------------------------------------------
# Library — supervisor core + AAPF generator + ROS layer + dev obstacle helper.
# ----------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/supervisor.cpp                # Core class (no ROS) — 5-state machine, P1/P2
  src/aapf_generator.cpp            # AAPF twist generator (real math)
  src/singularity_handler.cpp       # Core class (no ROS) — κ hysteresis, joint-hold coast
  src/aapf_supervisor_node.cpp      # ROS layer — Node class, plumbing
  src/aapf_supervisor_node_main.cpp # Composable registration only
  src/obstacle_manager.cpp          # Dev helper — spawn/remove obstacle (fixed)
  src/interactive_obstacle.cpp      # Dev helper — draggable obstacle (peer source)
  src/joint_velocity_estimator.cpp  # q̇ from joint-position finite-difference
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  control_msgs
  controller_manager_msgs
  shape_msgs
  visualization_msgs
  interactive_markers
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometric_shapes
  tf2_ros
  tf2_eigen
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# Register SupervisorNode as a composable node.
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "custom_ur_aapf_2::SupervisorNode"
  EXECUTABLE aapf_supervisor_node
)

# Register ObstacleManager as a composable node.
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "custom_ur_aapf_2::ObstacleManager"
  EXECUTABLE obstacle_manager_node
)

# Register InteractiveObstacle as a composable node.
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "custom_ur_aapf_2::InteractiveObstacle"
  EXECUTABLE interactive_obstacle_node
)

# Register JointVelocityEstimator as a composable node.
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "custom_ur_aapf_2::JointVelocityEstimator"
  EXECUTABLE joint_velocity_estimator_node
)

# ----------------------------------------------------------------------------
# Install
# ----------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp rclcpp_action rclcpp_components geometry_msgs sensor_msgs std_msgs
  std_srvs trajectory_msgs control_msgs controller_manager_msgs shape_msgs visualization_msgs
  interactive_markers moveit_msgs moveit_core
  moveit_ros_planning moveit_ros_planning_interface geometric_shapes
  tf2_ros tf2_eigen
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Pure-logic unit tests (no ROS spin needed).
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_singularity_handler test/test_singularity_handler.cpp)
  target_link_libraries(test_singularity_handler ${PROJECT_NAME})
endif()

ament_package()
