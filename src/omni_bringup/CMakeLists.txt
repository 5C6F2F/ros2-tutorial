cmake_minimum_required(VERSION 3.10)
project(omni_bringup)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_package(INSTALL_TO_SHARE config launch rviz)
