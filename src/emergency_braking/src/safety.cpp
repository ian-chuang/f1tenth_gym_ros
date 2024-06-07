// used https://github.com/CL2-UWaterloo/f1tenth_ws/tree/main/src/safety_node as reference

#include "emergency_braking/safety.hpp"

Safety::Safety() : Node("safety_node"), speed(0.0) {
    // Create publisher and subscriptions
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&Safety::scan_callback, this, _1));
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&Safety::odom_callback, this, _1));
}

void Safety::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    // Update speed
    this->speed = msg->twist.twist.linear.x;
}

void Safety::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    /// calculate TTC
    emergency_braking = false;
    for (std::size_t i = 0; i < scan_msg->ranges.size(); i++) {
        double r = scan_msg->ranges[i];
        if (std::isnan(r) || r > scan_msg->range_max || r < scan_msg->range_min) {
            continue;
        }
        double threshold = 1;  // To be tuned in real vehicle
        if (r / std::max(this->speed * std::cos(scan_msg->angle_min + (double)i * scan_msg->angle_increment), 0.001) < threshold) {
            emergency_braking = true;
            break;
        }
    }
    // Publish command to brake
    if (emergency_braking) {
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = 0.0;
        RCLCPP_INFO(this->get_logger(), "emergency brake engaged at speed '%f'", this->speed);  // Output to log;
        this->publisher_->publish(drive_msg);
    }
}
