cmake_minimum_required(VERSION 3.8)
project(lgdxrobot2_daemon)

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_NAME lgdxrobot2_daemon_node)
set(PROTO_NAME lgdxrobot2_daemon_proto)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#
# gRPC
#
#list(APPEND CMAKE_PREFIX_PATH "../third_party/grpc") 
#find_package(protobuf CONFIG REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)

add_library(${PROTO_NAME} 
  proto/RobotClientsService.proto
)
target_link_libraries(${PROTO_NAME} 
  PUBLIC
  protobuf::libprotobuf
  gRPC::grpc
  gRPC::grpc++
)
target_include_directories(${PROTO_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

get_target_property(grpc_cpp_plugin_location gRPC::grpc_cpp_plugin LOCATION)
protobuf_generate(TARGET ${PROTO_NAME} LANGUAGE cpp)
protobuf_generate(TARGET ${PROTO_NAME} LANGUAGE grpc GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc PLUGIN "protoc-gen-grpc=${grpc_cpp_plugin_location}")

#
# Node Msg
#
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutoTask.msg"
  "srv/AutoTaskNext.srv"
  "srv/AutoTaskAbort.srv"
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

#
# Node
#
add_subdirectory(../third_party/hwinfo hwinfo)
find_package(Boost REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(${EXECUTABLE_NAME} 
  src/CloudAdapter.cpp
  src/DaemonNode.cpp
  src/main.cpp 
  src/Navigation.cpp
  src/RealtimeExchange.cpp
  src/RobotStatus.cpp
  src/SerialPort.cpp
)
target_link_libraries(${EXECUTABLE_NAME} 
  ${PROTO_NAME}
  "${cpp_typesupport_target}"
  hwinfo
  #hwinfo::HWinfo
  Boost::headers
)
ament_target_dependencies(${EXECUTABLE_NAME}  
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2_geometry_msgs nav2_msgs nav2_util
)

install(TARGETS ${EXECUTABLE_NAME} DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROTO_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
