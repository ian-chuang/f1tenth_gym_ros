#include <gtest/gtest.h>
#include "obs_detect/obs_detect.h"
#include <rclcpp/rclcpp.hpp>

class OBS_DETECT_Test : public ::testing::Test
{
protected:
  std::shared_ptr<OBS_DETECT> node_;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<OBS_DETECT>();
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(OBS_DETECT_Test, ConstructorTest)
{
  EXPECT_EQ(node_->current_car_speed, 0.0);
  EXPECT_EQ(node_->collision_l, 3.0);
  EXPECT_FALSE(node_->use_coll_avoid);
  EXPECT_FALSE(node_->got_pose_flag);
  EXPECT_EQ(node_->collision_detect_counter, 0);
  EXPECT_EQ(node_->global_obs_detect_goal.size(), 3);
  EXPECT_EQ(node_->global_obs_detect_goal[0], 0.0);
  EXPECT_EQ(node_->global_obs_detect_goal[1], 0.0);
  EXPECT_EQ(node_->global_obs_detect_goal[2], 0.0);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
