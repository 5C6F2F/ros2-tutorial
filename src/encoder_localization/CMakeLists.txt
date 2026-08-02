cmake_minimum_required(VERSION 3.8)
project(encoder_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- ここから ---

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## FetchContent
include(FetchContent)

### nholthaus/units
FetchContent_Declare(
  units
  GIT_REPOSITORY https://github.com/nholthaus/units
  GIT_TAG        master
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(units)

# add executable
ament_auto_add_executable(encoder_localization src/node.cpp)
target_link_libraries(
  encoder_localization
  units
)

ament_auto_add_executable(cmd_vel_to_encoder src/cmd_vel_to_encoder.cpp)
target_link_libraries(
  cmd_vel_to_encoder
  units
)

# install
install(
  TARGETS encoder_localization cmd_vel_to_encoder
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# --- ここまで ---

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
