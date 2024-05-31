// code modified from https://github.com/ladavis4/F1Tenth_Final_Project_and_ICRA2022/tree/main/gap_follow

#include "gap_follow/gap_follow.hpp"
#include <sstream>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

ReactiveFollowGap::ReactiveFollowGap() : Node("reactive_node") {
    // Declare parameters
    this->declare_parameter("drive_topic", "/drive");
    this->declare_parameter("lidarscan_topic", "/scan");
    this->declare_parameter("decision_topic", "/use_obs_avoid");
    this->declare_parameter("pure_pursuit_velocity_topic", "pure_pursuit_velocity");
    this->declare_parameter("window_size", 3);
    this->declare_parameter("max_range_threshold", 7.0);
    this->declare_parameter("max_drive_range_threshold", 5.0);
    this->declare_parameter("car_width", 0.60);
    this->declare_parameter("angle_cutoff", 1.5);
    this->declare_parameter("disp_threshold", 0.4);
    this->declare_parameter("bubble_dist_threshold", 6.0);
    this->declare_parameter("velocity_scaling_factor", 5.0);
    this->declare_parameter("minimum_speed", 0.5);

    // read parameters
    std::string drive_topic = this->get_parameter("drive_topic").as_string();
    std::string lidarscan_topic = this->get_parameter("lidarscan_topic").as_string();
    std::string decision_topic = this->get_parameter("decision_topic").as_string();
    std::string pure_pursuit_velocity_topic = this->get_parameter("pure_pursuit_velocity_topic").as_string();
    window_size = this->get_parameter("window_size").as_int();
    max_range_threshold = this->get_parameter("max_range_threshold").as_double();
    max_drive_range_threshold = this->get_parameter("max_drive_range_threshold").as_double();
    car_width = this->get_parameter("car_width").as_double();
    angle_cutoff = this->get_parameter("angle_cutoff").as_double();
    disp_threshold = this->get_parameter("disp_threshold").as_double();
    bubble_dist_threshold = this->get_parameter("bubble_dist_threshold").as_double();
    velocity_scaling_factor = this->get_parameter("velocity_scaling_factor").as_double();
    minimum_speed = this->get_parameter("minimum_speed").as_double();

    /// Create ROS subscribers and publishers
    drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1);
    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 1, std::bind(&ReactiveFollowGap::lidar_callback, this, _1));
    gap_sub = this->create_subscription<std_msgs::msg::Bool>(decision_topic, 1, std::bind(&ReactiveFollowGap::gap_callback, this, _1));
    velocity_sub = this->create_subscription<std_msgs::msg::Float64>(pure_pursuit_velocity_topic, 1, std::bind(&ReactiveFollowGap::velocity_callback, this, _1));
}

void ReactiveFollowGap::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    if (use_gap==true){
        //Load in range information from message
        int num_readings = scan_msg->ranges.size();
        float ranges_raw[1081];
        copy(std::begin(scan_msg->ranges), std::end(scan_msg->ranges), std::begin(ranges_raw));

        //Create array of angles for the raw ranges
        std::vector<float> angles_raw;
        float cur_angle;
        float angle_increment = scan_msg->angle_increment;
        for(int i = 0; i < num_readings; i++){
            cur_angle = (scan_msg->angle_increment * i) + (scan_msg->angle_min);
            angles_raw.push_back(cur_angle);
        }

        //Find the start and end of the angle cutoff
        //Reduces the number of ranges looked at
        int cutoff_start_idx;
        int cutoff_end_idx;
        int num_readings_p;
        for(int i = 0; i < num_readings; i++){
            if (angles_raw[i] > (angle_cutoff * -1.0)){
                cutoff_start_idx = i;
                break;
            }
        }
        for(int i = 0; i < num_readings; i++){
            if (angles_raw[i] > (angle_cutoff)){
                cutoff_end_idx = i;
                break;
            }
        }
        num_readings_p = cutoff_end_idx - cutoff_start_idx;

        //Create new "processed" angle and ranges vectors as a subset of the raw ranges and angles
        std::vector<float> ranges_p(&ranges_raw[cutoff_start_idx], &ranges_raw[cutoff_end_idx]);
        std::vector<float> angles_p(&angles_raw[cutoff_start_idx], &angles_raw[cutoff_end_idx]);

        // Preprocess the lidar data
        preprocess_lidar(ranges_p, num_readings_p); //updates ranges_p

        // find gap
        int num_disp;
        std::vector<int> disp_idx;
        num_disp = find_disparities(disp_idx, ranges_p, num_readings_p);
        std::vector<float> ranges_p_clean = ranges_p;
        set_disparity(ranges_p, num_readings_p, disp_idx, num_disp, angle_increment, ranges_p_clean);  // Set all values at the disparity to the value of the closest point
        set_close_bubble(ranges_p, angles_p, num_readings_p, angle_increment);
        int *gap_idxes = find_max_gap(ranges_p, num_readings_p); //find the drive idx from the max gap

        // Find the drive angle
        float drive_angle;
        int drive_idx {0};
        drive_angle = find_drive_angle(ranges_p, angles_p, gap_idxes, drive_idx);

        // Check if the car is going to hit a wall
        bool going_to_hit=false;
        going_to_hit = corner_safety_check(ranges_raw, angles_raw, drive_angle, num_readings, angle_increment);

        // Publish Drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        if (going_to_hit==false){
            drive_msg.drive.steering_angle = drive_angle;
        }
        else{
            drive_msg.drive.steering_angle = 0;

        }
        // scale the drive vel based on the range in front of the car
        drive_msg.drive.speed = drive_speed_calc(ranges_p, angles_p, num_readings_p, pure_pursuit_velocity); 
        // publish the drive message
        drive_publisher->publish(drive_msg);
    }
}

void ReactiveFollowGap::gap_callback(const std_msgs::msg::Bool::ConstSharedPtr gap_bool) {
    // var to use gap follow or not
    use_gap = gap_bool->data;
}

void ReactiveFollowGap::velocity_callback(const std_msgs::msg::Float64::ConstSharedPtr val) {
    // get pure pursuit velocity
    pure_pursuit_velocity = val->data;
}

void ReactiveFollowGap::preprocess_lidar(std::vector<float>& ranges, int num_readings) {
    // update the ranges vector
    std::vector<float> old_ranges;
    old_ranges = ranges;

    // Setting each value to the mean over some window
    float running_mean = 0.0;
    ranges[0] = old_ranges[0];
    for(int i = 1; i < (num_readings - 1); i++){
        running_mean = 0.0;
        for(int j = -1; j < 2; j++){
            running_mean += old_ranges[i + j];
        }
        ranges[i] = running_mean/3.0;
        if (ranges[i] < 0){
            ranges[i] = 0;
        }
    }
    ranges[num_readings -1] = old_ranges[num_readings - 1];

    // Rejecting high values
    for(int i = 0; i < num_readings; i++) {
        if (ranges[i] > max_range_threshold) {
            ranges[i] = max_range_threshold;
        }
    }
}

int ReactiveFollowGap::find_disparities(std::vector<int>& disp_idx, std::vector<float>& ranges, int num_readings) {
    // Find the disparities in the ranges
    int disp_count = 0;
    for(int i = 1; i < num_readings; i++){
        if (abs(ranges[i] - ranges[i-1]) > disp_threshold){
            //there is a disparity
            if (ranges[i] < ranges[i -1]){
                disp_idx.push_back(i);
            } else{
                disp_idx.push_back(i-1);
            }
            disp_count += 1;
        }
    }
    return disp_count;
}

void ReactiveFollowGap::set_disparity(std::vector<float>& ranges, int num_points, std::vector<int>& disp_idx, int num_disp, float angle_increment, std::vector<float>& ranges_clean) {
    float theta;
    int n;
    float n_float;
    int bubble_idx;
    float bubble_dist;

    for (int i=0; i<num_disp; i++){
        bubble_idx = disp_idx[i];

        bubble_dist = ranges_clean[bubble_idx];

        if(bubble_dist > bubble_dist_threshold){
            continue;
        }

        theta = atan2((car_width /2.0), bubble_dist);
        n_float = theta/angle_increment;
        n = static_cast<int>(n_float);

        //Cases to fix out of bounds errors
        if (bubble_idx + n > num_points){
            n = num_points - bubble_idx;
        }
        if (bubble_idx - n < 0.0 ){
            n = bubble_idx;
        }

        //Set points within bubble to zero
        for (int j = 0; j < n + 1; j++){
            if (bubble_dist < ranges[bubble_idx + j]){
                ranges[bubble_idx + j] = bubble_dist;
            }

        }
        for (int j = 0; j < n + 1; j++){
            if (bubble_dist < ranges[bubble_idx - j]){
                ranges[bubble_idx - j] = bubble_dist;
            }
        }
    }
}

void ReactiveFollowGap::set_close_bubble(std::vector<float>& ranges, std::vector<float>& angles, int num_points, float angle_increment) {
    int close_idx;
    float low_val = 100.0;

    //Find the closest point
    for (int i = 0; i < num_points; i++){
        if (ranges[i] < low_val){
            low_val = ranges[i];
            close_idx = i;
        }
    }
    float theta;
    int n;
    float n_float;
    int bubble_idx;

    bubble_idx = close_idx;

    //Use trig to find number of points to eliminate
    theta = atan2((car_width /2.0), ranges[bubble_idx]);
    n_float = theta/angle_increment; 
    n = static_cast<int>(n_float);

    //Cases to fix out of bounds errors
    int n_up = n;
    int n_down = n;
    if (bubble_idx + n > num_points){
        n_up = num_points - bubble_idx;
    }
    if (bubble_idx - n < 0.0 ){
        n_down = bubble_idx;
    }

    //Set points within bubble to zero
    for (int j = 0; j < n_up + 1; j++){
        ranges[bubble_idx + j] = 0.0;
    }
    for (int j = 0; j < n_down + 1; j++){
        ranges[bubble_idx - j] = 0.0;
    }
}

int* ReactiveFollowGap::find_max_gap(std::vector<float>& ranges, int num_readings) {
    // Return the start index & end index of the max gap
    int current_gap_width = 0; // width of the current gap being tested
    int max_gap_width = 0; // largest gap found so far
    int gap_flag = 0; // 0 for no gap, 1 for gap
    int current_start_idx = 0; // start index for current gap being tested

    int start_idx = 0; // overall largest gap start
    int end_idx = 0; // overall largest gap end

    static int gap_idxes[2];

    for(int i = 0; i < num_readings; i++){
        if (ranges[i] > 0.0){
            if(gap_flag == 0){
                //New gap
                current_start_idx = i;
            }
            gap_flag = 1;
            current_gap_width +=1;
        } else {
            if(gap_flag == 1){
                //Gap ended
                if(current_gap_width > max_gap_width){
                    //New largest gap
                    start_idx = current_start_idx;
                    end_idx = i -1;
                    max_gap_width = current_gap_width;
                }
            }
            gap_flag = 0;
            current_gap_width = 0;
        }
    }
    if(current_gap_width > max_gap_width){
        start_idx = current_start_idx;
        end_idx = num_readings - 1;
    }

    gap_idxes[0] = start_idx;
    gap_idxes[1] = end_idx;
    return gap_idxes;
}

float ReactiveFollowGap::find_drive_angle(std::vector<float>& ranges, std::vector<float>& angles, int* gap_idx, int &drive_idx) {
    //Find the largest group of max range readings and drive to the center
    //Attempts to fix the side to side motion of the car
    //Find the deepest range reading
    float deepest_val = 0.0;
    int deepest_idx;
    float drive_angle;


    for(int i = gap_idx[0]; i<gap_idx[1]; i++){
        if(ranges[i] > deepest_val){
            deepest_val = ranges[i];
            deepest_idx = i;
        }
    }

    if (deepest_val > (max_drive_range_threshold - 0.1)){
        //Find the max gap of the max readings
        int current_gap_width = 0; // width of the current gap being tested
        int max_gap_width = 0; // largest gap found so far
        int gap_flag = 0; // 0 for no gap, 1 for gap
        int current_start_idx = 0; // start index for current gap being tested

        int start_idx = 0; // overall largest gap start
        int end_idx = 0; // overall largest gap end

        for(int i = gap_idx[0]; i < gap_idx[1]; i++){
            if (ranges[i] > (max_drive_range_threshold - 0.1)){
                if(gap_flag == 0){
                    //New gap
                    current_start_idx = i;
                }
                gap_flag = 1;
                current_gap_width +=1;
            } else {
                if(gap_flag == 1){
                    //Gap ended
                    if(current_gap_width > max_gap_width){
                        //New largest gap
                        start_idx = current_start_idx;
                        end_idx = i - 1;
                        max_gap_width = current_gap_width;
                    }
                }
                gap_flag = 0;
                current_gap_width = 0;
            }
        }
        if(current_gap_width > max_gap_width){
            start_idx = current_start_idx;
            end_idx = gap_idx[1] - 1;
        }
        drive_idx = ((end_idx - start_idx)/2) + start_idx;
        drive_angle = angles[drive_idx];
    } else {
        //Take the deepest point in the vector
        drive_angle = angles[deepest_idx];
        drive_idx = deepest_idx;
    }
    return drive_angle;
}

float ReactiveFollowGap::drive_speed_calc(std::vector<float>& ranges, std::vector<float>& angles, int num_readings, double max_drive_speed) {
    //Get average range in front facing window
    float drive_speed;

    float window_size = 0.174533; //10 degrees to both sides
    int window_start_idx;
    int window_end_idx;

    // Find the start and end idx of the window
    for (int i = 0; i<num_readings; i++){
        if (angles[i] > (window_size * -1)){
            window_start_idx = i;
            break;
        }
    }
    for (int i = num_readings; i > -1; i--){
        if (angles[i] < window_size){
            window_end_idx = i;
            break;
        }
    }

    // Get average over front facing window
    int num_window_readings = window_end_idx - window_start_idx;
    float num_window_readings_float = static_cast<float>(num_window_readings);
    float mean = 0.0;

    for (int i = window_start_idx; i <= window_end_idx; i++){
        mean += ranges[i];
    }
    mean = mean / num_window_readings_float;

    drive_speed = max_drive_speed * (1 - (1 / (1 + mean/velocity_scaling_factor)));

    // set a minimum speed
    if (drive_speed < minimum_speed){
        drive_speed = minimum_speed;
        RCLCPP_DEBUG(this->get_logger(), "Drive speed below minimum, setting to minimum of %f", minimum_speed);
    }
    RCLCPP_DEBUG(this->get_logger(), "max_drive_speed %f, drive_speed %f, drive_factor %f", max_drive_speed, drive_speed, (1 - (1 / (1 + mean/velocity_scaling_factor))));

    return drive_speed;
}

bool ReactiveFollowGap::corner_safety_check(float p_ranges[1080], std::vector<float> angles, float drive_angle, int num_readings, float angle_increment) {
    bool crashing = false;

    float safety_range_threshold = .5;
    float drive_activation_angle = .20632; //Set to 25 deg right now
    float window_size = 0.174533; //10 degrees
    float window_start = 1.5708;

    float num_window_readings = window_size / angle_increment;

    float window_mean = 0.0;

    if (abs(drive_angle) > drive_activation_angle && drive_angle < 0){
        //right turn
        //get average of right side
        for(int i = 0; i < num_readings; i++){
            if (angles[i] < (window_start * -1) && angles[i] > (window_start * -1 - window_size)){
                window_mean += p_ranges[i];
            }
        }
        window_mean = window_mean / num_window_readings;

        if (window_mean < safety_range_threshold){
            crashing = true;
        }
    }

    if (abs(drive_angle) > drive_activation_angle && drive_angle > 0){
        //left turn
        //get average of left side
        for(int i = 0; i < num_readings; i++){
            if (angles[i] > window_start  && angles[i] < window_start + window_size){
                window_mean += p_ranges[i];
            }
        }
        window_mean = window_mean / num_window_readings;
        if (window_mean < safety_range_threshold){
            crashing = true;
        }
    }
    return crashing;
}

