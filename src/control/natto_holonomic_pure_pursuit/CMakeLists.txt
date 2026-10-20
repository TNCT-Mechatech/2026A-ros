cmake_minimum_required(VERSION 3.8)
project(natto_holonomic_pure_pursuit)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "holonomic_pure_pursuit::holonomic_pure_pursuit"
    EXECUTABLE holonomic_pure_pursuit
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_path_geometry test/test_path_geometry.cpp)
    target_compile_features(test_path_geometry PRIVATE cxx_std_17)
    target_include_directories(test_path_geometry PRIVATE include)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
