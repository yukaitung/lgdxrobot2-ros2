cmake_minimum_required(VERSION 3.8)
project(lgdxrobot2_agent)

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_NAME lgdxrobot2_agent_node)
set(PROTO_NAME lgdxrobot2_agent_proto)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#
# Node
#
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
find_package(lgdxrobot_cloud_msgs REQUIRED)

include_directories(include)

add_library(lgdxrobot2_agent_component SHARED
  src/Agent.cpp
  src/Mcu.cpp
  src/Sensors.cpp
  src/Cloud.cpp
)
target_link_libraries(lgdxrobot2_agent_component
  "${cpp_typesupport_target}"
  Boost::headers
)
ament_target_dependencies(lgdxrobot2_agent_component
  geometry_msgs 
  lgdxrobot2_msgs
  lgdxrobot_cloud_msgs
  nav_msgs 
  rclcpp 
  rclcpp_components 
  sensor_msgs 
  std_msgs 
  tf2_geometry_msgs 
)

rclcpp_components_register_node(lgdxrobot2_agent_component
  PLUGIN "LgdxRobot2::Agent"
  EXECUTABLE lgdxrobot2_agent_node
)

ament_export_targets(export_lgdxrobot2_agent_component)
install(TARGETS lgdxrobot2_agent_component
  EXPORT export_lgdxrobot2_agent_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
