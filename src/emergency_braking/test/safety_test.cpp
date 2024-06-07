#include <gtest/gtest.h>
#include "emergency_braking/safety.hpp"

TEST(SafetyTest, ConstructorTest) {
  Safety safety;
  
  // check that publisher, scan_subscription and odom_subscription are not null
  ASSERT_NE(nullptr, safety.publisher_);
  ASSERT_NE(nullptr, safety.scan_subscription_);
  ASSERT_NE(nullptr, safety.odom_subscription_);
}

TEST(SafetyTest, OdomCallbackTest) {
  // Create Safety object
  Safety safety;

  // Create Odometry message
  nav_msgs::msg::Odometry::SharedPtr odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  odom_msg->twist.twist.linear.x = 5.0; // Set the linear velocity

  // Call the odom_callback function
  safety.odom_callback(odom_msg);

  // Check if the speed is updated correctly
  ASSERT_EQ(safety.speed, 5.0);
}

TEST(SafetyTest, BrakeTrueTest) {
  // Create Safety object
  Safety safety;
  safety.speed = 5.0; // Set the speed
  // Create LaserScan message
  sensor_msgs::msg::LaserScan::SharedPtr scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan_msg->ranges = {0.1,0.1,0.1,0.1,0.1}; // Set the ranges
  scan_msg->range_max = 10.0; // Set the range max
  scan_msg->range_min = 0.0; // Set the range min
  scan_msg->angle_min = -M_PI/2; // Set the angle min
  scan_msg->angle_max = M_PI/2; // Set the angle max
  scan_msg->angle_increment = M_PI/4; // Set the angle increment
  // Call the scan_callback function
  safety.scan_callback(scan_msg);
  // Check if the emergency braking is engaged
  ASSERT_TRUE(safety.emergency_braking);
}

TEST(SafetyTest, BrakeFalseTest) {
  // Create Safety object
  Safety safety;
  safety.speed = 0.1; // Set the speed
  // Create LaserScan message
  sensor_msgs::msg::LaserScan::SharedPtr scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan_msg->ranges = {3.0,3.0,3.0,3.0,3.0}; // Set the ranges
  scan_msg->range_max = 10.0; // Set the range max
  scan_msg->range_min = 0.0; // Set the range min
  scan_msg->angle_min = -M_PI/2; // Set the angle min
  scan_msg->angle_max = M_PI/2; // Set the angle max
  scan_msg->angle_increment = M_PI/4; // Set the angle increment
  // Call the scan_callback function
  safety.scan_callback(scan_msg);
  // Check if the emergency braking is engaged
  ASSERT_FALSE(safety.emergency_braking);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
