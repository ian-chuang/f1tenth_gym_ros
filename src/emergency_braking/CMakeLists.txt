cmake_minimum_required(VERSION 3.5)
project(emergency_braking)

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
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(emergency_braking src/safety_node.cpp src/safety.cpp)
ament_target_dependencies(emergency_braking 
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs
)

# Install Cpp executables
install(TARGETS
  emergency_braking
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/safety_test.cpp src/safety.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs
  )
  target_link_libraries(${PROJECT_NAME}_test )
endif()


ament_package()
