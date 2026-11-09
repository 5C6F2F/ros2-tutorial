cmake_minimum_required(VERSION 3.10)
project(omni_fake_hardware)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(fake_mcu src/fake_mcu.cpp)
ament_auto_add_executable(fake_lidar_localizer src/fake_lidar_localizer.cpp)

ament_auto_package()
