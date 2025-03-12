cmake_minimum_required(VERSION 3.8)
project(gripper_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)



# Add the executable
add_executable(gripper_service_server src/gripper_service_server.cpp)
ament_target_dependencies(gripper_service_server rclcpp example_interfaces sensor_msgs moveit_ros_planning_interface)

add_executable(basic_service src/basic_service.cpp)
ament_target_dependencies(basic_service rclcpp example_interfaces)

# Install the executable
install(TARGETS
  gripper_service_server
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  basic_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
