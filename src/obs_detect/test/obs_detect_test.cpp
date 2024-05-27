#include <gtest/gtest.h>
#include "obs_detect/obs_detect.h"

#include <gtest/gtest.h>

TEST(obs_detect, ConstructorTest)
{
  // Create an instance of PurePursuit
  OBS_DETECT obs_detect;
  EXPECT_EQ(obs_detect.current_car_speed, 0.0);
  EXPECT_EQ(obs_detect.collision_l, 3);
}

// Add more test cases as needed

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}