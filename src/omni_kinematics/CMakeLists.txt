cmake_minimum_required(VERSION 3.10)
project(omni_kinematics)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_omni_kinematics test/test_omni_kinematics.cpp)
  target_include_directories(test_omni_kinematics PRIVATE include)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
