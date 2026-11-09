cmake_minimum_required(VERSION 3.10)
project(omni_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(encoder_counts_listener src/encoder_counts_listener.cpp)
ament_auto_add_executable(encoder_odometry src/encoder_odometry.cpp)
ament_auto_add_executable(map_odom_corrector src/map_odom_corrector.cpp)

ament_auto_package()
