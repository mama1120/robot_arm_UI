cmake_minimum_required(VERSION 3.5)
project(rviz2_teach_plugin)

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(interactive_markers REQUIRED)
find_package(std_srvs REQUIRED) 




set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(include)
# Für service
include_directories(${robot_arm_getpose_INCLUDE_DIRS})


# Add the correct source file
add_library(${PROJECT_NAME} SHARED
  src/teach_plugin.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  pluginlib
  Qt5Core
  Qt5Gui
  Qt5Widgets
  interactive_markers
  std_srvs
)

# Export pluginlib descriptions
pluginlib_export_plugin_description_file(rviz_common rviz_plugin_description.xml)


install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
