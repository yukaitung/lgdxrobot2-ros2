cmake_minimum_required(VERSION 3.8)
project(lgdxrobot2_mcu)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED)

add_executable(mcu_node src/mcu_node.cpp)
ament_target_dependencies(mcu_node rclcpp std_msgs sensor_msgs)
target_link_libraries(mcu_node Boost::headers)

install(TARGETS
  mcu_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()