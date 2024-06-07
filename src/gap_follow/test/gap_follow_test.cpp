#include <gtest/gtest.h>
#include "gap_follow/gap_follow.hpp"

class GAP_FOLLOW_Test : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<ReactiveFollowGap>();
  }

  std::shared_ptr<ReactiveFollowGap> node_;
};

TEST_F(GAP_FOLLOW_Test, ConstructorTest)
{
  // check float within some epsilon
  EXPECT_NEAR(node_->window_size, 3, 0.0001);
  EXPECT_NEAR(node_->max_range_threshold, 7.0, 0.0001);
  EXPECT_NEAR(node_->max_drive_range_threshold, 5.0, 0.0001);
  EXPECT_NEAR(node_->car_width, 0.6, 0.0001);
  EXPECT_NEAR(node_->angle_cutoff, 1.5, 0.0001);
  EXPECT_NEAR(node_->disp_threshold, 0.4, 0.0001);
  EXPECT_NEAR(node_->bubble_dist_threshold, 6.0, 0.0001);
  EXPECT_NEAR(node_->velocity_scaling_factor, 5.0, 0.0001);
  EXPECT_NEAR(node_->minimum_speed, 0.5, 0.0001);
}

TEST_F(GAP_FOLLOW_Test, InitialPublisherState)
{
  ASSERT_NE(node_->drive_publisher, nullptr);
  ASSERT_NE(node_->scan_sub, nullptr);
  ASSERT_NE(node_->gap_sub, nullptr);
  ASSERT_NE(node_->velocity_sub, nullptr);
}

TEST_F(GAP_FOLLOW_Test, LidarCallbackTest)
{
  // Create a sample LaserScan message
  auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan_msg->ranges = std::vector<float>(1080, 1.0);
  ASSERT_NO_THROW(node_->lidar_callback(scan_msg));
}

TEST_F(GAP_FOLLOW_Test, GapCallbackTest)
{
  // Create a sample Bool message
  auto gap_msg = std::make_shared<std_msgs::msg::Bool>();
  gap_msg->data = true;
  ASSERT_NO_THROW(node_->gap_callback(gap_msg));

  // Assert that the use_gap flag is set to true
  EXPECT_TRUE(node_->use_gap);

  // Create another sample Bool message
  auto gap_msg2 = std::make_shared<std_msgs::msg::Bool>();
  gap_msg2->data = false;
  ASSERT_NO_THROW(node_->gap_callback(gap_msg2));

  // Assert that the use_gap flag is set to false
  EXPECT_FALSE(node_->use_gap);
}

TEST_F(GAP_FOLLOW_Test, VelocityCallbackTest)
{
  // Create a sample Float64 message
  auto velocity_msg = std::make_shared<std_msgs::msg::Float64>();
  velocity_msg->data = 2.5;
  ASSERT_NO_THROW(node_->velocity_callback(velocity_msg));

  // Assert that the pure_pursuit_velocity parameter is updated correctly
  EXPECT_NEAR(node_->pure_pursuit_velocity, 2.5, 0.0001);
}

TEST_F(GAP_FOLLOW_Test, PreprocessLidarTest)
{
  // Create a sample LaserScan message
  auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan_msg->ranges = {1.0, 2.0, 3.0, 4.0, 5.0};

  // Call the preprocess_lidar function
  node_->preprocess_lidar(scan_msg->ranges, scan_msg->ranges.size());

  // Check if the ranges have been preprocessed correctly
  EXPECT_FLOAT_EQ(scan_msg->ranges[0], 1.0);
  EXPECT_FLOAT_EQ(scan_msg->ranges[1], (1.0 + 2.0 + 3.0) / 3.0);
  EXPECT_FLOAT_EQ(scan_msg->ranges[2], (2.0 + 3.0 + 4.0) / 3.0);
  EXPECT_FLOAT_EQ(scan_msg->ranges[3], (3.0 + 4.0 + 5.0) / 3.0);
  EXPECT_FLOAT_EQ(scan_msg->ranges[4], 5.0);
}

TEST(ReactiveFollowGapTest, FindDisparitiesTest)
{
  ReactiveFollowGap node;

  // Create a sample LaserScan message
  auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan_msg->ranges = {1.0, 2.0, 3.0, 4.0, 5.0};

  // Call the lidar_callback function
  node.lidar_callback(scan_msg);

  // Check if the disparities are found correctly
  std::vector<int> disp_idx;
  int num_disp = node.find_disparities(disp_idx, scan_msg->ranges, scan_msg->ranges.size());
  EXPECT_EQ(num_disp, 4);
  EXPECT_EQ(disp_idx.size(), 4);
  EXPECT_EQ(disp_idx[0], 0);
  EXPECT_EQ(disp_idx[1], 1);
}

TEST(ReactiveFollowGapTest, SetDisparityTest)
{
  ReactiveFollowGap node;

  // Create sample input data
  std::vector<float> ranges = {1.0, 2.0, 3.0, 4.0, 5.0};
  int num_points = ranges.size();
  std::vector<int> disp_idx = {1, 3};
  int num_disp = disp_idx.size();
  float angle_increment = 0.1;
  std::vector<float> ranges_clean = ranges;

  // Call the set_disparity function
  node.set_disparity(ranges, num_points, disp_idx, num_disp, angle_increment, ranges_clean);

  // Check if the ranges have been updated correctly
  EXPECT_FLOAT_EQ(ranges[0], 1.0);
  EXPECT_FLOAT_EQ(ranges[1], 2.0);
  EXPECT_FLOAT_EQ(ranges[2], 2.0);
  EXPECT_FLOAT_EQ(ranges[3], 4.0);
  EXPECT_FLOAT_EQ(ranges[4], 5.0);
}

TEST(ReactiveFollowGapTest, SetCloseBubbleTest)
{
  ReactiveFollowGap node;

  // Create sample input data
  std::vector<float> ranges = {1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<float> angles = {-0.2, -0.1, 0.0, 0.1, 0.2};
  int num_points = ranges.size();
  float angle_increment = 0.1;

  // Call the set_close_bubble function
  node.set_close_bubble(ranges, angles, num_points, angle_increment);

  // Check if the ranges have been updated correctly
  EXPECT_FLOAT_EQ(ranges[0], 0.0);
  EXPECT_FLOAT_EQ(ranges[1], 0.0);
  EXPECT_FLOAT_EQ(ranges[2], 0.0);
  EXPECT_FLOAT_EQ(ranges[3], 4.0);
  EXPECT_FLOAT_EQ(ranges[4], 5.0);
}

TEST(ReactiveFollowGapTest, FindMaxGapTest) {
  // Create a ReactiveFollowGap object
  ReactiveFollowGap node;

  // Create a sample LaserScan message
  auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan_msg->ranges = {1.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0, 4.0, 0.0, 0.0, 5.0};

  // Call the lidar_callback function
  node.lidar_callback(scan_msg);

  // Find the start and end index of the max gap
  int* gap_idxes = node.find_max_gap(scan_msg->ranges, scan_msg->ranges.size());

  // Assert that the start and end index are correct
  EXPECT_EQ(gap_idxes[0], 7);
  EXPECT_EQ(gap_idxes[1], 8);
}

TEST(ReactiveFollowGapTest, FindDriveAngleTest1) {
  ReactiveFollowGap node;

  // Create sample input data
  std::vector<float> ranges = {1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<float> angles = {-0.2, -0.1, 0.0, 0.1, 0.2};
  int gap_idx[2] = {1, 3};
  int drive_idx;

  // Call the find_drive_angle function
  float drive_angle = node.find_drive_angle(ranges, angles, gap_idx, drive_idx);

  // Check if the drive angle is correct
  EXPECT_FLOAT_EQ(drive_angle, 0.0);
  // Check if the drive index is correct
  EXPECT_EQ(drive_idx, 2);
}

TEST(ReactiveFollowGapTest, DriveSpeedCalcTest)
{
  ReactiveFollowGap node;

  // Create sample input data
  std::vector<float> ranges = {1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<float> angles = {-0.2, -0.1, 0.0, 0.1, 0.2};
  int num_readings = ranges.size();
  double max_drive_speed = 10.0;

  // Call the drive_speed_calc function
  float drive_speed = node.drive_speed_calc(ranges, angles, num_readings, max_drive_speed);

  // Check if the drive speed is calculated correctly
  EXPECT_FLOAT_EQ(drive_speed, 4.1176472);
}

TEST(ReactiveFollowGapTest, CornerSafetyCheckTest) {
  ReactiveFollowGap node;

  // Create sample input data
  float p_ranges[1080] = {1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<float> angles = {-0.2, -0.1, 0.0, 0.1, 0.2};
  float drive_angle = 0.15;
  int num_readings = 5;
  float angle_increment = 0.1;

  // Call the corner_safety_check function
  bool crashing = node.corner_safety_check(p_ranges, angles, drive_angle, num_readings, angle_increment);

  // Check if the result is correct
  EXPECT_FALSE(crashing);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}


