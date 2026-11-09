cmake_minimum_required(VERSION 3.10)
project(omni_localization)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(encoder_counts_listener src/encoder_counts_listener.cpp)
ament_auto_add_executable(encoder_odometry src/encoder_odometry.cpp)
ament_auto_add_executable(map_odom_corrector src/map_odom_corrector.cpp)

ament_auto_package()
