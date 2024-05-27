#include <gtest/gtest.h>
#include "pure_pursuit/pure_pursuit.hpp"

#include <gtest/gtest.h>

TEST(pure_pursuit, ConstructorTest)
{
  // Create an instance of PurePursuit
  PurePursuit purePursuit;
  EXPECT_EQ(purePursuit.min_lookahead, 0.5);
  EXPECT_EQ(purePursuit.max_lookahead, 1.0);
  EXPECT_EQ(purePursuit.lookahead_ratio, 8.0);
  EXPECT_EQ(purePursuit.K_p, 0.5);
  EXPECT_EQ(purePursuit.steering_limit, 25.0);
  EXPECT_EQ(purePursuit.velocity_percentage, 0.6);
}

TEST(pure_pursuit, ToRadiansTest) {
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