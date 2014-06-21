#include <ros/ros.h>
#include <gtest/gtest.h>
#include "bluefox2/bluefox2_ros.h"

TEST(CameraTest, construstor) {
  ros::NodeHandle nh("~");
  bluefox2::CameraRos camera_ros(nh);
  camera_ros.init();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtest_mono");
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
