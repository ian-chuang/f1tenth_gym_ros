#include <gtest/gtest.h>
#include "pure_pursuit/pure_pursuit.hpp"

TEST(PurePursuitTest, ConstructorTest)
{
  // Create an instance of PurePursuit
  PurePursuit purePursuit;
}

TEST(PurePursuitTest, ToRadiansTest) {
  PurePursuit pure_pursuit;

  // Test with negative angle
  double degrees = -90.0;
  double expected_result = -1.5708; // Approximately -pi/2
  double result = pure_pursuit.to_radians(degrees);
  EXPECT_NEAR(result, expected_result, 0.1);
}

// Add more test cases as needed

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}