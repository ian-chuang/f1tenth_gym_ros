cmake_minimum_required(VERSION 3.5)
project(gap_follow)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(PACKAGE
  ament_cmake
  ament_cmake_python
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(reactive_node src/reactive_node.cpp)
ament_target_dependencies(reactive_node
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs
)
add_executable(reactive_node_opp src/reactive_node_opp.cpp)
ament_target_dependencies(reactive_node_opp
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs
)
add_executable(null_node src/null_node.cpp)
ament_target_dependencies(null_node
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs
)

add_executable(reactive_node_solo src/reactive_node_solo.cpp)
ament_target_dependencies(reactive_node_solo
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs
)

# Install Cpp executables
install(TARGETS
  reactive_node
  reactive_node_opp
  null_node
  reactive_node_solo
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/reactive_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()