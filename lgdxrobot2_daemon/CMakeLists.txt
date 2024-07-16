cmake_minimum_required(VERSION 3.8)
project(lgdxrobot2_daemon)

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_NAME lgdxrobot2_daemon_node)
set(PROTO_NAME lgdxrobot2_daemon_proto)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Grpc
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(grpc REQUIRED grpc)
pkg_search_module(grpc++ REQUIRED grpc++)

add_library(${PROTO_NAME} 
  proto/RobotClientService.proto
)
target_link_libraries(${PROTO_NAME} 
  PUBLIC
  protobuf::libprotobuf
  grpc
  grpc++
)
target_include_directories(${PROTO_NAME}  PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

protobuf_generate(TARGET ${PROTO_NAME}  LANGUAGE cpp)
protobuf_generate(TARGET ${PROTO_NAME}  LANGUAGE grpc GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc PLUGIN "protoc-gen-grpc=/usr/bin/grpc_cpp_plugin")

# Node
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutoTask.msg"
  "srv/AutoTaskNext.srv"
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

include_directories(include/${PROJECT_NAME})

add_executable(${EXECUTABLE_NAME} 
  src/main.cpp 
  src/DaemonNode.cpp
)
target_link_libraries(${EXECUTABLE_NAME} 
  ${PROTO_NAME}
  "${cpp_typesupport_target}"
)
ament_target_dependencies(${EXECUTABLE_NAME}  
  rclcpp
)

install(TARGETS ${EXECUTABLE_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
