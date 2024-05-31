// built off of https://github.com/ladavis4/F1Tenth_Final_Project_and_ICRA2022/tree/main/obs_detect_pkg
#include "obs_detect/obs_detect.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <Eigen/Geometry>
#include "std_msgs/msg/bool.hpp"
#include <math.h>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

OBS_DETECT::~OBS_DETECT()
{
    RCLCPP_INFO(rclcpp::get_logger("OBS_DETECT"), "%s\n", "OBS_DETECT shutting down");
}

// Constructor of the OBS_DETECT class
OBS_DETECT::OBS_DETECT() : rclcpp::Node("obs_detect_node")
{
    // initialise parameters
    this->declare_parameter("spline_file_name", "/home/car1/f1tenth_ws/src/darc_f1tenth_system/f1tenth_stack/racelines/struggle_screen_recording.csv");
    this->declare_parameter("coll_grid_topic", "/coll_grid_pub_rviz");
    this->declare_parameter("coll_path_topic", "/coll_path_pub_rviz");
    this->declare_parameter("use_avoid_topic", "/use_obs_avoid");
    this->declare_parameter("gap_theta_topic", "/gap_thetas");
    this->declare_parameter("scan_topic", "/scan");
    this->declare_parameter("drive_topic", "/drive");
    this->declare_parameter("pose_topic", "/pf/pose/odom");
    // get params
    std::string spline_file_name = this->get_parameter("spline_file_name").as_string();
    std::string coll_grid_topic = this->get_parameter("coll_grid_topic").as_string();
    std::string coll_path_topic = this->get_parameter("coll_path_topic").as_string();
    std::string use_avoid_topic = this->get_parameter("use_avoid_topic").as_string();
    std::string gap_theta_topic = this->get_parameter("gap_theta_topic").as_string();
    std::string scan_topic = this->get_parameter("scan_topic").as_string();
    std::string drive_topic = this->get_parameter("drive_topic").as_string();
    std::string pose_topic = this->get_parameter("pose_topic").as_string();

    // ROS publishers
    grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(coll_grid_topic, 1);
    path_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(coll_path_topic, 1);
    use_avoid_pub = this->create_publisher<std_msgs::msg::Bool>(use_avoid_topic, 1);
    gap_theta_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(gap_theta_topic, 1);

    // ROS subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 1, std::bind(&OBS_DETECT::scan_callback, this, std::placeholders::_1));
    drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1, std::bind(&OBS_DETECT::drive_callback, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(pose_topic, 1, std::bind(&OBS_DETECT::pose_callback, this, std::placeholders::_1));

    // Read in spline points
    std::vector<float> row;
    std::string line, number;
    std::fstream file(spline_file_name, ios::in);
    if (file.is_open())
    {
        while (getline(file, line))
        {
            row.clear();
            stringstream str(line);
            while (getline(str, number, ','))
                row.push_back(std::stof(number));
            spline_points.push_back(row);
        }
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("OBS_DETECT"), "Error opening file\n");
    }

    // Initialize pose
    q.x() = 0;
    q.y() = 0;
    q.z() = 0;
    q.w() = 1;
    rotation_mat = q.normalized().toRotationMatrix();
    current_car_speed = 0.0;
    collision_l = 3.0;
}

void OBS_DETECT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    // Receive a scan message and update the occupancy grid
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //    listed_data: A new occupancy grid
    //    checks if we need to use gap follow

    if (got_pose_flag)
    {
        // Find the position of the local goal
        global_obs_detect_goal = spline_points[goal_spline_idx];
        Eigen::Vector3d local_point((global_obs_detect_goal[0] - current_car_pose.pose.pose.position.x), (global_obs_detect_goal[1] - current_car_pose.pose.pose.position.y), 0);
        Eigen::Vector3d local_goal = rotation_mat.inverse() * local_point;
        int x_goal = (local_goal[0] / resolution) + center_x;
        int y_goal = (local_goal[1] / resolution) + center_y;

        // Build the occupancy grid
        std::vector<signed char> occugrid_flat(occu_grid_y_size * occu_grid_x_size);
        int x_scan;
        int y_scan;
        for (int i = 0; i < scan_msg->ranges.size(); i++)
        {
            if (std::isnan(scan_msg->ranges[i]) == false && std::isinf(scan_msg->ranges[i]) == false && scan_msg->ranges[i] != 0)
            {
                // Find the location of the scan in occugrid x and y coordinates
                x_scan = scan_msg->ranges[i] * cos(scan_msg->angle_increment * i + scan_msg->angle_min) / resolution;
                y_scan = scan_msg->ranges[i] * sin(scan_msg->angle_increment * i + scan_msg->angle_min) / resolution;

                if (scan_padding == true)
                {
                    for (int j = -1 + x_scan; j < 2 + x_scan; j++)
                    { // 8
                        for (int k = -1 + y_scan; k < 2 + y_scan; k++)
                        {
                            if (j + center_x > 0 && j + center_x < occu_grid_x_size)
                            {
                                if (k + center_y > 0 && k + center_y < occu_grid_y_size)
                                {
                                    occugrid_flat[((k + center_y) * occu_grid_x_size) + (j + center_x)] = 100;
                                }
                            }
                        }
                    }
                }
                else
                {
                    if (x_scan + center_x > 0 && x_scan + center_x < occu_grid_x_size && y_scan + center_y > 0 && y_scan + center_y < occu_grid_y_size)
                    {
                        occugrid_flat[((y_scan + center_y) * occu_grid_x_size) + (x_scan + center_x)] = 100;
                    }
                }
            }
        }

        // publish occupancy grid
        if (publish_rviz == true)
        {
            publish_grid(occugrid_flat);
        }

        // check if obstacle in grid
        check_to_activate_obs_avoid(occugrid_flat);
    }
}

void OBS_DETECT::check_to_activate_obs_avoid(std::vector<signed char> &occugrid_flat)
{
    // only run once pose is received
    if (got_pose_flag == true)
    {
        // calculate num iterations
        int max_spline_idx = spline_points.size();
        int increment = 10;
        int iterations = 0;
        bool run_check = true;
        if (goal_spline_idx - car_spline_idx > 0)
        {
            iterations = (goal_spline_idx - car_spline_idx) / increment;
        }
        else if (goal_spline_idx - car_spline_idx < 0)
        {
            iterations = (max_spline_idx - car_spline_idx + goal_spline_idx) / increment;
        }
        else
        {
            run_check = false;
        }

        // run obs avoid check
        std::vector<std::vector<int>> grid_interp_points;
        int goal_point[2];
        spline_point2occu_coordinate(car_spline_idx, goal_point);
        int origin_point[2] = {center_x, center_y};
        std::vector<std::vector<int>> segment_interp_points;
        segment_interp_points = draw_connecting_line(origin_point, goal_point);
        grid_interp_points.insert(grid_interp_points.end(), segment_interp_points.begin(), segment_interp_points.end());
        if (run_check == true)
        {
            int origin_idx;
            int goal_idx = car_spline_idx;
            for (int b = 0; b <= iterations; b += 1)
            {
                origin_idx = goal_idx;
                goal_idx += increment;
                if (goal_idx >= max_spline_idx)
                {
                    goal_idx = goal_idx - max_spline_idx;
                }
                if (origin_idx >= max_spline_idx)
                {
                    origin_idx = origin_idx - max_spline_idx;
                }
                if (b == iterations)
                {
                    goal_idx = goal_spline_idx;
                }

                // If on first iteration, connect car to spline
                int goal_point[2];
                int origin_point[2];
                spline_point2occu_coordinate(goal_idx, goal_point);

                if (b == 0)
                {
                    origin_point[0] = center_x;
                    origin_point[1] = center_y;
                }
                else
                {
                    spline_point2occu_coordinate(origin_idx, origin_point);
                }

                std::vector<std::vector<int>> segment_interp_points;
                segment_interp_points = draw_connecting_line(origin_point, goal_point);
                grid_interp_points.insert(grid_interp_points.end(), segment_interp_points.begin(), segment_interp_points.end());
            }
        }

        // update path data
        std::vector<signed char> path_data(occu_grid_y_size * occu_grid_x_size);
        for (int i = 0; i < grid_interp_points.size(); i++)
        {
            if (grid_interp_points[i][1] >= 0 && grid_interp_points[i][0] >= 0)
            {
                if (((grid_interp_points[i][1]) * occu_grid_x_size) + (grid_interp_points[i][0]) < path_data.size())
                {
                    path_data[((grid_interp_points[i][1]) * occu_grid_x_size) + (grid_interp_points[i][0])] = 100;
                }
            }
        }

        // Check if there is a collision!
        use_coll_avoid = false;
        for (int i = 0; i < path_data.size(); i++)
        {
            if (path_data[i] == 100 && i < occugrid_flat.size() && occugrid_flat[i] == 100)
            {
                use_coll_avoid = true;
                break;
            }
        }

        // publish the racline path on the grid
        if (publish_rviz == true)
        {
            publish_path(path_data);
        }

        // publish obs avoid msg
        auto use_coll_avoid_msg = std_msgs::msg::Bool();
        if (use_coll_avoid == true)
        {
            use_coll_avoid_msg.data = use_coll_avoid;
            collision_detect_counter = 0;
        }
        else
        {
            collision_detect_counter += 1;
            if (collision_detect_counter < collision_loop_threshold)
            {
                use_coll_avoid_msg.data = true;
            }
            else
            {
                use_coll_avoid_msg.data = false;
            }
        }
        use_avoid_pub->publish(use_coll_avoid_msg);
    }
}

void OBS_DETECT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
{
    // The pose callback when subscribed to particle filter's inferred pose
    // Check to see if we need gap follow

    // get msg
    current_car_pose = *pose_msg;
    q.x() = current_car_pose.pose.pose.orientation.x;
    q.y() = current_car_pose.pose.pose.orientation.y;
    q.z() = current_car_pose.pose.pose.orientation.z;
    q.w() = current_car_pose.pose.pose.orientation.w;
    rotation_mat = q.normalized().toRotationMatrix();

    // update waypoint idx of car from current pose
    car_spline_idx = find_spline_index(current_car_pose.pose.pose.position.x, current_car_pose.pose.pose.position.y);
    goal_spline_idx = find_obs_detect_goal_idx(collision_l, spline_points, car_spline_idx);

    // update flag
    got_pose_flag = true;
}

void OBS_DETECT::drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr drive_msg)
{
    // The drive callback gets the published drive message
    ackermann_msgs::msg::AckermannDriveStamped msg = *drive_msg;
    // update vals
    current_car_speed = msg.drive.speed;
    collision_l = current_car_speed * collision_time_buffer;
}

// get occupancy grid coordinate from spline point
int *OBS_DETECT::spline_point2occu_coordinate(int spline_idx, int *occu_point)
{
    float x_local_w = spline_points[spline_idx][0] - current_car_pose.pose.pose.position.x;
    float y_local_w = spline_points[spline_idx][1] - current_car_pose.pose.pose.position.y;

    Eigen::Vector3d p_local_w(x_local_w, y_local_w, 0);
    Eigen::Vector3d p_local_c = rotation_mat.inverse() * p_local_w;
    occu_point[0] = (p_local_c[0] / resolution) + center_x;
    occu_point[1] = (p_local_c[1] / resolution) + center_y;

    return occu_point;
}

/*
This function takes the origin point and goal point in occugrid coordinates and returns a 2D array segment_interp_points using bresenhams_line algorithm.
The connecting line is widened to ensure contact with other points
*/
std::vector<std::vector<int>> OBS_DETECT::draw_connecting_line(int origin_point[2], int goal_point[2])
{
    std::vector<std::vector<int>> segment_interp_points;
    segment_interp_points = bresenhams_line_algorithm(goal_point, origin_point);

    if (path_line_padding == true)
    {
        int add_val_x = 1;
        int add_val_y = 1;
        int x_val;
        int y_val;
        int size_val = segment_interp_points.size();

        // Add point +1/-1 in the y direction
        for (int i = 0; i < size_val; i++)
        {
            x_val = segment_interp_points[i][0];
            y_val = segment_interp_points[i][1] + 1;
            if (y_val > 0 && y_val < occu_grid_y_size)
            {
                std::vector<int> added_point{x_val, y_val};
                segment_interp_points.push_back(added_point);
            }
            y_val = segment_interp_points[i][1] - 1;
            if (y_val > 0 && y_val < occu_grid_y_size)
            {
                std::vector<int> added_point{x_val, y_val};
                segment_interp_points.push_back(added_point);
            }
        }
    }
    return segment_interp_points;
}

std::vector<std::vector<int>> OBS_DETECT::bresenhams_line_algorithm(int goal_point[2], int origin_point[2])
{
    try
    {
        int x1 = origin_point[0];
        int y1 = origin_point[1];
        int x2 = goal_point[0];
        int y2 = goal_point[1];

        int y_diff = y2 - y1;
        int x_diff = x2 - x1;

        bool swapped = false;

        if (abs(y_diff) >= abs(x_diff))
        {
            swapped = true;
            x1 = origin_point[1];
            y1 = origin_point[0];
            x2 = goal_point[1];
            y2 = goal_point[0];
        }

        int intermediate;
        if (x1 > x2)
        {
            intermediate = x1;
            x1 = x2;
            x2 = intermediate;

            intermediate = y1;
            y1 = y2;
            y2 = intermediate;
        }

        y_diff = y2 - y1;
        x_diff = x2 - x1;

        int error = int(x_diff / 2);
        float ystep = -1;
        if (y1 < y2)
        {
            ystep = 1;
        }

        int y = y1;
        std::vector<std::vector<int>> output;
        for (int x = x1; x < x2 + 1; x++)
        {
            std::vector<int> coords{x, y};
            if (abs(y_diff) > abs(x_diff))
            {
                coords[0] = y;
                coords[1] = x;
            }
            output.push_back(coords);
            error -= abs(y_diff);
            if (error < 0)
            {
                y += ystep;
                error += x_diff;
            }
        }

        if (swapped == true)
        {
            std::vector<std::vector<int>> newoutput;
            for (int i = 0; i < output.size(); i++)
            {
                std::vector<int> newcoords{output[i][1], output[i][0]};
                newoutput.push_back(newcoords);
            }
            return newoutput;
        }

        else
        {
            return output;
        }

        return output;
    }
    catch (...)
    {
        std::cout << "bresenhams failed" << std::endl;
    }
}

int OBS_DETECT::find_spline_index(float x, float y)
{
    /*
    Returns the index of the closest point on the spline to (x,y)
    */
    float spline_index = -10000;
    float min_val = 1000;

    for (int i = 0; i < spline_points.size(); i++)
    {
        float dist = sqrt(pow(abs(x - spline_points[i][0]), 2) + pow(abs(y - spline_points[i][1]), 2));
        if (dist < min_val)
        {
            spline_index = i;
            min_val = dist;
        }
    }
    return spline_index;
}

int OBS_DETECT::find_obs_detect_goal_idx(float l_dist, std::vector<std::vector<float>> spline_points, int car_idx)
{
    float total_dist = 0.0;
    int goal_point_idx;
    int current_idx = car_idx;
    int next_idx = car_idx + 1;

    for (int i = 0; i < spline_points.size(); i++)
    {
        if (current_idx >= spline_points.size())
        {
            current_idx = 0;
        }
        if (next_idx >= spline_points.size())
        {
            current_idx = 0;
        }

        total_dist += sqrt(pow(abs(spline_points[current_idx][0] - spline_points[next_idx][0]), 2) + pow(abs(spline_points[current_idx][1] - spline_points[next_idx][1]), 2));
        if (total_dist > l_dist)
        {
            goal_point_idx = i;
            break;
        }
        current_idx += 1;
        next_idx += 1;
        if (next_idx >= spline_points.size())
        {
            next_idx = 0;
        }
    }
    goal_point_idx += car_idx;
    if (goal_point_idx >= spline_points.size())
    {
        goal_point_idx -= spline_points.size();
    }
    return goal_point_idx;
}

void OBS_DETECT::publish_grid(std::vector<signed char> &occugrid_flat)
{
    // Publish the occupancy grid
    auto new_grid = nav_msgs::msg::OccupancyGrid();
    new_grid.info.resolution = resolution;
    new_grid.info.width = occu_grid_x_size;
    new_grid.info.height = occu_grid_y_size;
    std::string frame_id = "map";
    new_grid.header.frame_id = frame_id;
    new_grid.header.stamp = rclcpp::Clock().now();
    new_grid.info.origin = current_car_pose.pose.pose;
    Eigen::Vector3d occu_grid_shift(center_x * resolution, center_y * resolution, 0);
    Eigen::Vector3d shift_in_global_coords = rotation_mat * occu_grid_shift;
    new_grid.info.origin.position.x -= shift_in_global_coords[0];
    new_grid.info.origin.position.y -= shift_in_global_coords[1];
    new_grid.data = occugrid_flat;
    grid_pub->publish(new_grid);
}

void OBS_DETECT::publish_path(std::vector<signed char> &occugrid_flat)
{
    // Publish the occupancy grid
    auto new_grid = nav_msgs::msg::OccupancyGrid();
    new_grid.info.resolution = resolution;
    new_grid.info.width = occu_grid_x_size;
    new_grid.info.height = occu_grid_y_size;
    std::string frame_id = "map";
    new_grid.header.frame_id = frame_id;
    new_grid.header.stamp = rclcpp::Clock().now();
    new_grid.info.origin = current_car_pose.pose.pose;
    Eigen::Vector3d occu_grid_shift(center_x * resolution, center_y * resolution, 0);
    Eigen::Vector3d shift_in_global_coords = rotation_mat * occu_grid_shift;
    new_grid.info.origin.position.x -= shift_in_global_coords[0];
    new_grid.info.origin.position.y -= shift_in_global_coords[1];
    new_grid.data = occugrid_flat;
    path_pub->publish(new_grid);
}