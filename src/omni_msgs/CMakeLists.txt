cmake_minimum_required(VERSION 3.10)
project(omni_msgs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_auto_generate_interfaces()

ament_auto_package()
