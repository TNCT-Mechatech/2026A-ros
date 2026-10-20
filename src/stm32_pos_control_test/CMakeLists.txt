cmake_minimum_required(VERSION 3.8)
project(stm32_pos_control_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(debug_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(natto_robot_hardware REQUIRED)
find_package(yaml-cpp REQUIRED)

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

add_executable(stm32_pos_control_test src/main.cpp)

target_include_directories(stm32_pos_control_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/stm32_pos_control_test
)

target_compile_features(stm32_pos_control_test PRIVATE cxx_std_17)
target_link_libraries(stm32_pos_control_test serial_bridge yaml-cpp)
ament_target_dependencies(stm32_pos_control_test debug_msgs rclcpp std_msgs natto_robot_hardware)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(multi_stm32_integration test/test_multi_stm32.py
    TIMEOUT 60
    ENV STM32_TEST_EXECUTABLE=$<TARGET_FILE:stm32_pos_control_test>)
endif()

install(TARGETS stm32_pos_control_test
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/multi_stm32_control DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
