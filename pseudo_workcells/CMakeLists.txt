cmake_minimum_required(VERSION 3.8)
project(pseudo_workcells)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(workcell_triggers REQUIRED)

include_directories(
  include
)

add_executable(pseudo_workcells
  src/pseudo_workcells.cpp
  src/main.cpp
)

add_executable(pseudo_dispenser
  src/pseudo_dispenser.cpp
)

add_executable(pseudo_ingestor
  src/pseudo_ingestor.cpp
)

ament_target_dependencies(pseudo_workcells
  workcell_triggers
  rclcpp
)

ament_target_dependencies(pseudo_dispenser
  workcell_triggers
  rclcpp
)

ament_target_dependencies(pseudo_ingestor
  workcell_triggers
  rclcpp
)


# target_link_libraries(pseudo_workcells
#   workcell_triggers::workcell_triggers)

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

install(TARGETS
  pseudo_workcells
  pseudo_dispenser
  pseudo_ingestor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
