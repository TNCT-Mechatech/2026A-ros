cmake_minimum_required(VERSION 3.8)
project(inject2026A)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###############
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(natto_system_monitor REQUIRED)
find_package(natto_robot_hardware REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

###############
# Add Serialbridge
set(SERIAL_BRIDGE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../SerialBridge")
add_library(serial_bridge_transport STATIC
  ${SERIAL_BRIDGE_DIR}/src/source/LinuxHardwareSerial.cpp
  ${SERIAL_BRIDGE_DIR}/src/source/SerialBridge.cpp
  ${SERIAL_BRIDGE_DIR}/src/source/CobsSerial.cpp
  ${SERIAL_BRIDGE_DIR}/src/source/Message.cpp
  ${SERIAL_BRIDGE_DIR}/src/source/cobs.cpp
)
target_compile_features(serial_bridge_transport PRIVATE cxx_std_17)


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


ament_target_dependencies(main rclcpp rcl_interfaces std_msgs sensor_msgs geometry_msgs natto_system_monitor natto_robot_hardware)
target_compile_features(main PRIVATE cxx_std_17)

foreach(node ${node_list})
#  add_executable(${node} src/${node}.cpp src/trapezoidal_acc.cpp)
#  ament_target_dependencies(${node} rclcpp std_msgs sensor_msgs geometry_msgs)
  target_include_directories(${node} PRIVATE
    "${SERIAL_BRIDGE_DIR}/src"
    "${rclcpp_INCLUDE_DIRS}"
    "${std_msgs_INCLUDE_DIRS}"
  )
  target_link_libraries(${node}
    serial_bridge_transport
    yaml-cpp
    "${rclcpp_LIBRARIES}"
    "${std_msgs_LIBRARIES}"
  )
endforeach()

###############
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

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
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rack_controller test/test_rack_controller.cpp)
  target_compile_features(test_rack_controller PRIVATE cxx_std_17)
  target_link_libraries(test_rack_controller yaml-cpp)
  ament_add_gtest(test_rack_count_controller test/test_rack_count_controller.cpp)
  target_compile_features(test_rack_count_controller PRIVATE cxx_std_17)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(rack_serial_integration test/test_rack_serial.py
    TIMEOUT 60 ENV INJECT_TEST_EXECUTABLE=$<TARGET_FILE:main>)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
