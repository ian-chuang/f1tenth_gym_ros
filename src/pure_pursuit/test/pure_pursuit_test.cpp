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

TEST(pure_pursuit, ToRadiansTest0) {
  PurePursuit pure_pursuit;

  double degrees = 0.0;
  double radians = 0.0;
  double result = pure_pursuit.to_radians(degrees);
  EXPECT_NEAR(result, radians, 0.1);
}

TEST(pure_pursuit, ToRadiansTest1) {
  PurePursuit pure_pursuit;

  double degrees = 90.0;
  double radians = 1.5708; // Approximately pi/2
  double result = pure_pursuit.to_radians(degrees);
  EXPECT_NEAR(result, radians, 0.1);
}

TEST(pure_pursuit, ToRadiansTest2) {
  PurePursuit pure_pursuit;

  double degrees = -90.0;
  double radians = -1.5708; // Approximately -pi/2
  double result = pure_pursuit.to_radians(degrees);
  EXPECT_NEAR(result, radians, 0.1);
}

TEST(pure_pursuit, ToRadiansTest3) {
  PurePursuit pure_pursuit;

  double degrees = 45;
  double radians = 0.7854; // Approximately pi/4
  double result = pure_pursuit.to_radians(degrees);
  EXPECT_NEAR(result, radians, 0.1);
}

TEST(pure_pursuit, ToRadiansTest4) {
  PurePursuit pure_pursuit;

  double degrees = -45;
  double radians = -0.7854; // Approximately -pi/4
  double result = pure_pursuit.to_radians(degrees);
  EXPECT_NEAR(result, radians, 0.1);
}

TEST(pure_pursuit, ToRadiansTest5) {
  PurePursuit pure_pursuit;

  double degrees = 180;
  double radians = 3.1416; // Approximately pi
  double result = pure_pursuit.to_radians(degrees);
  EXPECT_NEAR(result, radians, 0.1);
}

TEST(pure_pursuit, ToRadiansTest6) {
  PurePursuit pure_pursuit;

  double degrees = -180;
  double radians = -3.1415; // Approximately -pi
  double result = pure_pursuit.to_radians(degrees);
  EXPECT_NEAR(result, radians, 0.1);
}

TEST(pure_pursuit, ToRadiansTest7) {
  PurePursuit pure_pursuit;

  double degrees = 360;
  double radians = 6.2832; // Approximately pi
  double result = pure_pursuit.to_radians(degrees);
  EXPECT_NEAR(result, radians, 0.1);
}

TEST(pure_pursuit, ToRadiansTest8) {
  PurePursuit pure_pursuit;

  double degrees = -360;
  double radians = -6.2832; // Approximately -pi
  double result = pure_pursuit.to_radians(degrees);
  EXPECT_NEAR(result, radians, 0.1);
}

TEST(pure_pursuit, ToDegreeTest0) {
  PurePursuit pure_pursuit;

  double radians = 0.0;
  double degrees = 0.0;
  double result = pure_pursuit.to_degrees(radians);
  EXPECT_NEAR(result, degrees, 0.1);
}

TEST(pure_pursuit, ToDegreeTest1) {
  PurePursuit pure_pursuit;

  double radians = 1.5708; // Approximately pi/2
  double degrees = 90.0;
  double result = pure_pursuit.to_degrees(radians);
  EXPECT_NEAR(result, degrees, 0.1);
}

TEST(pure_pursuit, ToDegreeTest2) {
  PurePursuit pure_pursuit;

  double radians = -1.5708; // Approximately -pi/2
  double degrees = -90.0;
  double result = pure_pursuit.to_degrees(radians);
  EXPECT_NEAR(result, degrees, 0.1);
}

TEST(pure_pursuit, ToDegreeTest3) {
  PurePursuit pure_pursuit;

  double radians = 0.7854; // Approximately pi/4
  double degrees = 45;
  double result = pure_pursuit.to_degrees(radians);
  EXPECT_NEAR(result, degrees, 0.1);
}

TEST(pure_pursuit, ToDegreeTest4) {
  PurePursuit pure_pursuit;

  double radians = -0.7854; // Approximately -pi/4
  double degrees = -45;
  double result = pure_pursuit.to_degrees(radians);
  EXPECT_NEAR(result, degrees, 0.1);
}

TEST(pure_pursuit, ToDegreeTest5) {
  PurePursuit pure_pursuit;

  double radians = 3.1416; // Approximately pi
  double degrees = 180;
  double result = pure_pursuit.to_degrees(radians);
  EXPECT_NEAR(result, degrees, 0.1);
}

TEST(pure_pursuit, ToDegreeTest6) {
  PurePursuit pure_pursuit;

  double radians = -3.1415; // Approximately -pi
  double degrees = -180;
  double result = pure_pursuit.to_degrees(radians);
  EXPECT_NEAR(result, degrees, 0.1);
}

TEST(pure_pursuit, ToDegreeTest7) {
  PurePursuit pure_pursuit;

  double radians = 6.2832; // Approximately pi
  double degrees = 360;
  double result = pure_pursuit.to_degrees(radians);
  EXPECT_NEAR(result, degrees, 0.1);
}

TEST(pure_pursuit, ToDegreeTest8) {
  PurePursuit pure_pursuit;

  double radians = -6.2832; // Approximately -pi
  double degrees = -360;
  double result = pure_pursuit.to_degrees(radians);
  EXPECT_NEAR(result, degrees, 0.1);
}


// Add more test cases as needed

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}