#include <ros/ros.h>
#include <gtest/gtest.h>
#include "bluefox2/camera.h"

TEST(CameraTest, construstor)
{
  ros::NodeHandle nh("~");
  bluefox2::Camera camera(nh);
  EXPECT_EQ(10.0, camera.fps());
  EXPECT_TRUE(camera.ok());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtest_mono");
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
