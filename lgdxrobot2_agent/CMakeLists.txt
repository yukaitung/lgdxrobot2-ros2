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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lgdxrobot2_msgs REQUIRED)

include_directories(include)

add_executable(${EXECUTABLE_NAME} 
  src/Agent.cpp
  src/main.cpp
  src/Mcu.cpp
  src/Sensors.cpp
)
target_link_libraries(${EXECUTABLE_NAME} 
  "${cpp_typesupport_target}"
  Boost::headers
)
ament_target_dependencies(${EXECUTABLE_NAME}  
  geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2_geometry_msgs lgdxrobot2_msgs
)

install(TARGETS ${EXECUTABLE_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
