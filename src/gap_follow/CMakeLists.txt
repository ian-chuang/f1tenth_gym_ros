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
add_executable(gap_follow src/gap_follow_node.cpp src/gap_follow.cpp)
ament_target_dependencies(gap_follow
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs
)


# Install Cpp executables
install(TARGETS
  gap_follow
  DESTINATION lib/${PROJECT_NAME})




ament_package()
