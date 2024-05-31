# Documentation for `obs_detect.cpp`

## Overview

The `obs_detect.cpp` file implements the `OBS_DETECT` class, which is responsible for obstacle detection and avoidance for a robot or autonomous vehicle. The class subscribes to various sensor topics, processes the incoming data, and publishes occupancy grids and obstacle avoidance commands.

## Constructor and Destructor

### `OBS_DETECT::OBS_DETECT()`

- Initializes the ROS node and declares parameters for topic names and file paths.
- Initializes ROS publishers and subscribers.
- Reads spline points from a file to be used for path planning.
- Initializes pose and rotation matrix variables.

### `OBS_DETECT::~OBS_DETECT()`

- Logs a message indicating that the `OBS_DETECT` object is shutting down.

## Callbacks

### `void OBS_DETECT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)`

- Processes incoming laser scan messages to update the occupancy grid.
- Checks if the car's pose has been received before processing.
- Computes the local goal position and updates the occupancy grid based on the scan data.
- Publishes the updated occupancy grid.
- Checks for obstacles in the grid and decides whether to activate obstacle avoidance.

### `void OBS_DETECT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)`

- Updates the car's pose based on incoming odometry messages.
- Computes the current spline index and goal spline index based on the car's position.
- Sets a flag indicating that the pose has been received.

### `void OBS_DETECT::drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr drive_msg)`

- Updates the car's speed and collision distance based on incoming drive messages.

## Utility Functions

### `int* OBS_DETECT::spline_point2occu_coordinate(int spline_idx, int *occu_point)`

- Converts a spline point index to occupancy grid coordinates.
- Uses the car's pose and rotation matrix to transform the spline point to local coordinates.

### `std::vector<std::vector<int>> OBS_DETECT::draw_connecting_line(int origin_point[2], int goal_point[2])`

- Draws a line between two points in the occupancy grid using Bresenham's line algorithm.
- Optionally adds padding to the line to make it wider.

### `std::vector<std::vector<int>> OBS_DETECT::bresenhams_line_algorithm(int goal_point[2], int origin_point[2])`

- Implements Bresenham's line algorithm to generate points between two grid coordinates.
- Handles cases where the line has a steep slope by swapping coordinates.

### `int OBS_DETECT::find_spline_index(float x, float y)`

- Finds the index of the closest spline point to the given coordinates (x, y).

### `int OBS_DETECT::find_obs_detect_goal_idx(float l_dist, std::vector<std::vector<float>> spline_points, int car_idx)`

- Finds the goal index for obstacle detection based on the car's current position and a specified distance.

## Publishing Functions

### `void OBS_DETECT::publish_grid(std::vector<signed char> &occugrid_flat)`

- Publishes the occupancy grid to a ROS topic.
- Uses the car's pose and rotation matrix to set the grid's origin.

### `void OBS_DETECT::publish_path(std::vector<signed char> &occugrid_flat)`

- Publishes the planned path on the occupancy grid to a ROS topic.
- Uses the car's pose and rotation matrix to set the grid's origin.

## ROS Parameters

- `spline_file_name`: Path to the file containing spline points.
- `coll_grid_topic`: Topic name for publishing the collision grid.
- `coll_path_topic`: Topic name for publishing the collision path.
- `use_avoid_topic`: Topic name for publishing the obstacle avoidance command.
- `gap_theta_topic`: Topic name for publishing gap angles.
- `scan_topic`: Topic name for subscribing to laser scan messages.
- `drive_topic`: Topic name for subscribing to drive commands.
- `pose_topic`: Topic name for subscribing to odometry messages.

## Usage

1. **Initialization**: Instantiate the `OBS_DETECT` class and initialize the ROS node.
2. **Subscription**: Subscribe to the necessary topics (laser scan, odometry, and drive commands).
3. **Processing**: The node processes incoming messages, updates the occupancy grid, and checks for obstacles.
4. **Publishing**: The node publishes the updated occupancy grid and obstacle avoidance commands based on the processed data.

This documentation provides an overview of the functionalities and components of the `obs_detect.cpp` file, explaining the purpose and operation of each method and the overall workflow of the `OBS_DETECT` class.
