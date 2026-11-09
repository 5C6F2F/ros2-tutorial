cmake_minimum_required(VERSION 3.10)
project(omni_navigation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(path_generator src/path_generator.cpp)
ament_auto_add_executable(path_follower src/path_follower.cpp)
ament_auto_add_executable(navigation_manager src/navigation_manager.cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_math test/test_path_math.cpp)
  target_include_directories(test_path_math PRIVATE include ${omni_kinematics_INCLUDE_DIRS})
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
