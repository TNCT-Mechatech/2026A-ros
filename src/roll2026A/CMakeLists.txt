cmake_minimum_required(VERSION 3.8)
project(roll2026A)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###############
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

###############
# Add SerialBridge
set(SERIAL_BRIDGE_DIR "${CMAKE_SOURCE_DIR}/SerialBridge")
add_library(serial_bridge STATIC
  "${SERIAL_BRIDGE_DIR}/src/source/CANSerialBridge.cpp"
  "${SERIAL_BRIDGE_DIR}/src/source/CobsSerial.cpp"
  "${SERIAL_BRIDGE_DIR}/src/source/InoHardwareSerial.cpp"
  "${SERIAL_BRIDGE_DIR}/src/source/LinuxHardwareSerial.cpp"
  "${SERIAL_BRIDGE_DIR}/src/source/MbedHardwareSerial.cpp"
  "${SERIAL_BRIDGE_DIR}/src/source/Message.cpp"
  "${SERIAL_BRIDGE_DIR}/src/source/SerialBridge.cpp"
  "${SERIAL_BRIDGE_DIR}/src/source/STM32HardwareSerial.cpp"
  "${SERIAL_BRIDGE_DIR}/src/source/cobs.cpp"
)
target_include_directories(serial_bridge PUBLIC "${SERIAL_BRIDGE_DIR}/src")

###############
# include custom header files
include_directories(
  include
  include/${PROJECT_NAME}
)

###############
# Add ROS nodes
set(node_list main)

add_executable(main
  src/main.cpp
  )


ament_target_dependencies(main rclcpp std_msgs sensor_msgs geometry_msgs)

foreach(node ${node_list})
#  add_executable(${node} src/${node}.cpp src/trapezoidal_acc.cpp)
#  ament_target_dependencies(${node} rclcpp std_msgs sensor_msgs geometry_msgs)
  target_include_directories(${node} PRIVATE
    "${SERIAL_BRIDGE_DIR}/src"
    "${rclcpp_INCLUDE_DIRS}"
    "${std_msgs_INCLUDE_DIRS}"
  )
  target_link_libraries(${node}
    serial_bridge
    "${rclcpp_LIBRARIES}"
    "${std_msgs_LIBRARIES}"
  )
endforeach()

###############
# Install launch files when the package provides any.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

###############



install(TARGETS
  ${node_list}
  DESTINATION lib/${PROJECT_NAME})




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
