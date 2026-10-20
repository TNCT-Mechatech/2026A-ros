cmake_minimum_required(VERSION 3.8)
project(param_checker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(debug_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(natto_robot_hardware REQUIRED)

set(SERIAL_BRIDGE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../SerialBridge)

add_library(serial_bridge STATIC
  ${SERIAL_BRIDGE_DIR}/src/source/SerialBridge.cpp
  ${SERIAL_BRIDGE_DIR}/src/source/CobsSerial.cpp
  ${SERIAL_BRIDGE_DIR}/src/source/Message.cpp
  ${SERIAL_BRIDGE_DIR}/src/source/cobs.cpp
  ${SERIAL_BRIDGE_DIR}/src/source/LinuxHardwareSerial.cpp
)

target_include_directories(serial_bridge PUBLIC
  ${SERIAL_BRIDGE_DIR}/src
)

target_compile_features(serial_bridge PUBLIC cxx_std_17)

add_executable(param_checker src/main.cpp)

target_include_directories(param_checker PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/param_checker
  ${CMAKE_CURRENT_SOURCE_DIR}/include/vrft_auto_tuner
  ${CMAKE_CURRENT_SOURCE_DIR}/include/aw-library
)

target_compile_features(param_checker PRIVATE cxx_std_17)
target_link_libraries(param_checker serial_bridge yaml-cpp)
ament_target_dependencies(param_checker rclcpp sensor_msgs std_msgs debug_msgs natto_robot_hardware)

add_executable(position_checker src/position_main.cpp)
target_include_directories(position_checker PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(position_checker PRIVATE cxx_std_17)
target_link_libraries(position_checker serial_bridge yaml-cpp)
ament_target_dependencies(position_checker rclcpp sensor_msgs std_msgs debug_msgs natto_robot_hardware)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_single_motor test/test_single_motor.cpp)
  target_include_directories(test_single_motor PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(test_single_motor PRIVATE cxx_std_17)
  target_link_libraries(test_single_motor yaml-cpp)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(single_motor_pipeline test/test_single_motor_pipeline.py
    TIMEOUT 90
    ENV VELOCITY_TEST_EXECUTABLE=$<TARGET_FILE:param_checker>
        POSITION_TEST_EXECUTABLE=$<TARGET_FILE:position_checker>)
  ament_add_gtest(test_position_vrft test/test_position_vrft.cpp)
  target_include_directories(test_position_vrft PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(test_position_vrft PRIVATE cxx_std_17)
endif()

install(TARGETS param_checker position_checker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
