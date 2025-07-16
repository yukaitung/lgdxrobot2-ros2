cmake_minimum_required(VERSION 3.8)
project(lgdxrobot2_webots)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lgdxrobot2_agent REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(webots_ros2_driver REQUIRED)

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver lgdxrobot2_driver.xml)

include_directories(include/lgdxrobot2_webots)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/LgdxRobot2Driver.cpp
)
ament_target_dependencies(
  ${PROJECT_NAME}
  geometry_msgs
  lgdxrobot2_agent
  nav_msgs
  pluginlib
  rclcpp
  std_msgs
  tf2_geometry_msgs
  webots_ros2_driver
)

install(TARGETS 
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY protos resource worlds maps
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
