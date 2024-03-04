cmake_minimum_required(VERSION 3.8)
project(lgdxrobot2_mcu)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED)

include_directories(include/lgdxrobot2_mcu)

add_executable(lgdxrobot2_mcu_node 
  src/main.cpp 
  src/McuNode.cpp 
  src/SerialPort.cpp
)
ament_target_dependencies(lgdxrobot2_mcu_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2_geometry_msgs)
target_link_libraries(lgdxrobot2_mcu_node Boost::headers)

install(TARGETS lgdxrobot2_mcu_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()