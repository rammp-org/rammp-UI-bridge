cmake_minimum_required(VERSION 3.8)
project(rammp_ui_bridge_streaming)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

add_library(rmss_protocol src/rmss_protocol.cpp)
target_include_directories(rmss_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(stream_sender src/stream_sender.cpp)
target_include_directories(stream_sender PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(stream_sender PUBLIC rmss_protocol nlohmann_json::nlohmann_json)

add_executable(camera_stream_node src/camera_stream_node.cpp)
target_link_libraries(camera_stream_node PRIVATE stream_sender)
ament_target_dependencies(camera_stream_node PUBLIC rclcpp sensor_msgs)

install(TARGETS
  rmss_protocol
  stream_sender
  camera_stream_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rmss_protocol test/test_rmss_protocol.cpp)
  target_link_libraries(test_rmss_protocol rmss_protocol)
endif()

ament_package()
