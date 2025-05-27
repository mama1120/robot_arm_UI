cmake_minimum_required(VERSION 3.5)
project(rviz2_teach_plugin)

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(interactive_markers REQUIRED)
find_package(std_srvs REQUIRED) 
find_package(geometry_msgs REQUIRED)  
find_package(rviz_services REQUIRED)

# Setup Qt Automoc, Autorcc and Include directories
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Include directories
include_directories(include)
include_directories(${rviz_services_INCLUDE_DIRS})

# Add the library target
add_library(${PROJECT_NAME} SHARED
  src/teach_plugin.cpp
  include/rviz2_teach_plugin/teach_plugin.hpp
)

# Set dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_rendering
  pluginlib
  Qt5Core
  Qt5Gui
  Qt5Widgets
  interactive_markers
  std_srvs
  geometry_msgs
  rviz_services
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
