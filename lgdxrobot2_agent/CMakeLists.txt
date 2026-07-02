cmake_minimum_required(VERSION 3.20)
project(lgdxrobot2_agent)

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_NAME lgdxrobot2_agent_component)
set(EXPORT_EXECUTABLE_NAME export_lgdxrobot2_agent_proto)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#
# Node
#
if (POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lgdxrobot2_msgs REQUIRED)

include_directories(include)

add_library(${EXECUTABLE_NAME} SHARED
  src/Agent.cpp
  src/Mcu.cpp
  src/Sensors.cpp
)
target_link_libraries(${EXECUTABLE_NAME}
  "${cpp_typesupport_target}"
  Boost::headers
)
target_link_libraries(${EXECUTABLE_NAME}
  geometry_msgs::geometry_msgs
  lgdxrobot2_msgs::lgdxrobot2_msgs
  nav_msgs::nav_msgs
  rclcpp::rclcpp
  rclcpp_components::component
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  tf2_geometry_msgs::tf2_geometry_msgs
)

rclcpp_components_register_node(${EXECUTABLE_NAME}
  PLUGIN "LgdxRobot2::Agent"
  EXECUTABLE lgdxrobot2_agent_node
)

ament_export_targets(${EXPORT_EXECUTABLE_NAME})
install(TARGETS ${EXECUTABLE_NAME}
  EXPORT ${EXPORT_EXECUTABLE_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
