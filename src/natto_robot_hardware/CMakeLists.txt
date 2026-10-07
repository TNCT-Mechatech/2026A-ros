cmake_minimum_required(VERSION 3.8)
project(natto_robot_hardware)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(debug_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(natto_system_monitor REQUIRED)
find_package(yaml-cpp REQUIRED)

set(SERIAL_BRIDGE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../SerialBridge)

message(STATUS "SERIAL_BRIDGE_DIR = ${SERIAL_BRIDGE_DIR}")

if(NOT EXISTS ${SERIAL_BRIDGE_DIR}/src/SerialBridge.hpp)
  message(FATAL_ERROR "SerialBridge.hpp not found: ${SERIAL_BRIDGE_DIR}/src/SerialBridge.hpp")
endif()

add_library(serial_bridge_transport STATIC
  ${SERIAL_BRIDGE_DIR}/src/source/LinuxHardwareSerial.cpp
  ${SERIAL_BRIDGE_DIR}/src/source/SerialBridge.cpp
  ${SERIAL_BRIDGE_DIR}/src/source/CobsSerial.cpp
  ${SERIAL_BRIDGE_DIR}/src/source/Message.cpp
  ${SERIAL_BRIDGE_DIR}/src/source/cobs.cpp
)
target_compile_features(serial_bridge_transport PRIVATE cxx_std_17)

add_executable(serial_motor_bridge
  src/serial_motor_bridge.cpp
)

target_compile_features(serial_motor_bridge PRIVATE cxx_std_17)

target_include_directories(serial_motor_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SERIAL_BRIDGE_DIR}/src
)

ament_target_dependencies(serial_motor_bridge
  rclcpp
  sensor_msgs
  std_msgs
  debug_msgs
  diagnostic_msgs
  natto_system_monitor
)

target_link_libraries(serial_motor_bridge
  serial_bridge_transport
  yaml-cpp
)

install(TARGETS
  serial_motor_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_serial_protocol test/test_serial_protocol.cpp)
  target_compile_features(test_serial_protocol PRIVATE cxx_std_17)
  target_include_directories(test_serial_protocol PRIVATE include ${SERIAL_BRIDGE_DIR}/src)
  target_link_libraries(test_serial_protocol serial_bridge_transport)
endif()

ament_package()