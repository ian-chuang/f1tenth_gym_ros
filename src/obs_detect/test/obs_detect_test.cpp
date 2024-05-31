#include <gtest/gtest.h>
#include "obs_detect/obs_detect.h"

class OBS_DETECT_Test : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<OBS_DETECT>();
  }

  std::shared_ptr<OBS_DETECT> node_;
};

TEST_F(OBS_DETECT_Test, ConstructorTest)
{
  EXPECT_EQ(node_->current_car_speed, 0.0);
  EXPECT_EQ(node_->collision_l, 3.0);
  EXPECT_EQ(node_->goal_spline_idx, 100);
  EXPECT_FALSE(node_->got_pose_flag);
  EXPECT_FALSE(node_->use_coll_avoid);
  EXPECT_EQ(node_->collision_detect_counter, 0);
}

TEST_F(OBS_DETECT_Test, InitialPublisherState)
{
  ASSERT_NE(node_->grid_pub, nullptr);
  ASSERT_NE(node_->path_pub, nullptr);
  ASSERT_NE(node_->use_avoid_pub, nullptr);
  ASSERT_NE(node_->gap_theta_pub, nullptr);
}

TEST_F(OBS_DETECT_Test, InitialSubscriberState)
{
  ASSERT_NE(node_->pose_sub_, nullptr);
  ASSERT_NE(node_->scan_sub_, nullptr);
  ASSERT_NE(node_->drive_sub_, nullptr);
}

TEST_F(OBS_DETECT_Test, PublishGridTest)
{
  std::vector<signed char> occugrid_flat(node_->occu_grid_x_size * node_->occu_grid_y_size, 0);
  ASSERT_NO_THROW(node_->publish_grid(occugrid_flat));
}

TEST_F(OBS_DETECT_Test, PublishPathTest)
{
  std::vector<signed char> path_grid_flat(node_->occu_grid_x_size * node_->occu_grid_y_size, 0);
  ASSERT_NO_THROW(node_->publish_path(path_grid_flat));
}

TEST_F(OBS_DETECT_Test, ScanCallbackTest)
{
  auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan_msg->ranges = std::vector<float>(1080, 1.0);
  ASSERT_NO_THROW(node_->scan_callback(scan_msg));
}

TEST_F(OBS_DETECT_Test, PoseCallbackTest)
{
  auto pose_msg = std::make_shared<nav_msgs::msg::Odometry>();
  pose_msg->pose.pose.position.x = 1.0;
  pose_msg->pose.pose.position.y = 2.0;
  pose_msg->pose.pose.orientation.w = 1.0;
  ASSERT_NO_THROW(node_->pose_callback(pose_msg));
}

TEST_F(OBS_DETECT_Test, DriveCallbackTest)
{
  auto drive_msg = std::make_shared<ackermann_msgs::msg::AckermannDriveStamped>();
  drive_msg->drive.speed = 3.0;
  ASSERT_NO_THROW(node_->drive_callback(drive_msg));
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
