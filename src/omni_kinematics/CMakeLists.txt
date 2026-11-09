cmake_minimum_required(VERSION 3.10)
project(omni_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_auto_add_gtest(test_omni_kinematics test/test_omni_kinematics.cpp)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
