#ifndef SAFETY_HPP
#define SAFETY_HPP

// used https://github.com/CL2-UWaterloo/f1tenth_ws/tree/main/src/safety_node as reference

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class Safety : public rclcpp::Node {
public:
    Safety();

    double speed;
    bool emergency_braking;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
};

#endif // SAFETY_HPP