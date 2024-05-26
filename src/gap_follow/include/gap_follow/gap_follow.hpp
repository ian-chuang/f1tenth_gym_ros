#ifndef REACTIVE_FOLLOW_GAP_HPP
#define REACTIVE_FOLLOW_GAP_HPP

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include <fstream>
#include <iostream>

class ReactiveFollowGap : public rclcpp::Node {
public:
    ReactiveFollowGap();
private:

    // gap params
    int window_size = 3; 
    float max_range_threshold = 10.0; 
    float max_drive_range_threshold = 5.0;
    float velocity_scaling_factor = 5.0;
    float car_width = .60; 
    float minimum_speed = 0.5;
    float angle_cutoff = 1.5; 
    float disp_threshold = .4;
    float bubble_dist_threshold = 6; 
    std::vector<float> velocity_points; 
    float pure_pursuit_velocity = 0; 
    bool use_gap = false;

    // pub/sub
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gap_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub;

    // methods
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    void preprocess_lidar(std::vector<float>& ranges, int num_readings);
    int find_disparities(std::vector<int>& disp_idx, std::vector<float>& ranges, int num_readings);
    int* find_max_gap(std::vector<float>& ranges, int num_readings);
    float find_drive_angle(std::vector<float>& ranges, std::vector<float>& angles, int* gap_idx, int &drive_idx);
    void set_disparity(std::vector<float>& ranges, int num_points, std::vector<int>& disp_idx, int num_disp, float angle_increment, std::vector<float>& ranges_clean);
    void set_close_bubble(std::vector<float>& ranges, std::vector<float>& angles, int num_points, float angle_increment);
    float drive_speed_calc(std::vector<float>& ranges, std::vector<float>& angles, int num_readings, double max_drive_speed);
    bool corner_safety_check(float p_ranges[1080], std::vector<float> angles, float drive_angle, int num_readings, float angle_increment);
    void gap_callback(const std_msgs::msg::Bool::ConstSharedPtr gap_bool);
    void velocity_callback(const std_msgs::msg::Float64::ConstSharedPtr val);
};

#endif // REACTIVE_FOLLOW_GAP_HPP
