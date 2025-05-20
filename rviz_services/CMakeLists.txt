cmake_minimum_required(VERSION 3.8)
project(rviz_services)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# Generate service messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetPoseRobotArm.srv"
  "srv/MoveLinear.srv"
  "srv/MoveToPose.srv"
  DEPENDENCIES std_msgs
)

# Export runtime dependency for the generated interfaces
ament_export_dependencies(rosidl_default_runtime)

# Include generated service headers
include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# ----------------------------
# GetPoseRobotArm Service Server
# ----------------------------
add_executable(get_pose_server src/get_pose_server.cpp)
ament_target_dependencies(get_pose_server 
  rclcpp 
  rosidl_default_runtime 
  sensor_msgs
)
# Link the generated type support library:
target_link_libraries(get_pose_server rviz_services__rosidl_typesupport_cpp)

# ----------------------------
# MoveLinear Service Server
# ----------------------------
add_executable(move_linear_service src/move_linear_service.cpp)
ament_target_dependencies(move_linear_service 
  rclcpp 
  rosidl_default_runtime
)
# Link the generated type support library:
target_link_libraries(move_linear_service rviz_services__rosidl_typesupport_cpp)

# Install the generated service headers
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# Install the executables
install(TARGETS
  get_pose_server
  move_linear_service
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
