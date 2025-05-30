cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rviz_services REQUIRED)
find_package(control_msgs REQUIRED)


# Include directories
include_directories(include)

add_executable(joint_position_receiver src/joint_position_receiver.cpp)
ament_target_dependencies(joint_position_receiver rclcpp moveit_msgs rclcpp_action)

install(TARGETS
joint_position_receiver
  DESTINATION lib/${PROJECT_NAME})

add_executable(multiple_positions src/multiple_positions.cpp)
ament_target_dependencies(multiple_positions rclcpp sensor_msgs moveit_ros_planning_interface)

install(TARGETS multiple_positions
  DESTINATION lib/${PROJECT_NAME})

  add_executable(ik_demo src/ik_demo.cpp)
ament_target_dependencies(ik_demo rclcpp sensor_msgs moveit_ros_planning_interface)

install(TARGETS ik_demo
  DESTINATION lib/${PROJECT_NAME})

    add_executable(ik_demo_wrist src/ik_demo_wrist.cpp)
ament_target_dependencies(ik_demo_wrist rclcpp sensor_msgs moveit_ros_planning_interface)

install(TARGETS ik_demo_wrist
  DESTINATION lib/${PROJECT_NAME})

  add_executable(forward_kinematic src/forward_kinematic.cpp)
ament_target_dependencies(forward_kinematic rclcpp sensor_msgs moveit_ros_planning_interface)

install(TARGETS forward_kinematic
  DESTINATION lib/${PROJECT_NAME})

add_executable(joint_listener src/joint_listener.cpp)
ament_target_dependencies(joint_listener rclcpp sensor_msgs moveit_ros_planning_interface control_msgs)
  
install(TARGETS joint_listener
  DESTINATION lib/${PROJECT_NAME})


add_executable(move_home src/move_home.cpp)
ament_target_dependencies(move_home rclcpp sensor_msgs moveit_ros_planning_interface)

install(TARGETS move_home
  DESTINATION lib/${PROJECT_NAME})

add_executable(move_X_Z src/move_X_Z.cpp)
ament_target_dependencies(move_X_Z rclcpp sensor_msgs moveit_ros_planning_interface)

install(TARGETS move_X_Z
  DESTINATION lib/${PROJECT_NAME})

add_executable(move_to_pose src/move_to_pose.cpp)
ament_target_dependencies(move_to_pose rclcpp sensor_msgs moveit_ros_planning_interface)

install(TARGETS move_to_pose
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(send_all_joints src/send_all_joints.cpp)
ament_target_dependencies(send_all_joints rclcpp moveit_msgs rclcpp_action)

install(TARGETS
  send_all_joints
  DESTINATION lib/${PROJECT_NAME})

add_executable(robot_arm_execution src/robot_arm_execution.cpp)
target_include_directories(robot_arm_execution PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS robot_arm_execution
DESTINATION lib/${PROJECT_NAME})

target_compile_features(robot_arm_execution PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  robot_arm_execution
  "moveit_ros_planning_interface"
  "rclcpp"
  example_interfaces
)

target_link_libraries(move_X_Z
  rviz_services::rviz_services__rosidl_typesupport_cpp
)
target_link_libraries(move_to_pose
  rviz_services::rviz_services__rosidl_typesupport_cpp
)
include_directories(
  ${rviz_services_INCLUDE_DIRS}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
