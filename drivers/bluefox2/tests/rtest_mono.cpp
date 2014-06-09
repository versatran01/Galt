#include <ros/ros.h>
#include <gtest/gtest.h>
#include "bluefox2/camera.h"

TEST(CameraTest, construstor)
{
    ros::NodeHandle nh;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtest_mono");
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
