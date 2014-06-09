#include <ros/ros.h>
#include <gtest/gtest.h>
#include "bluefox2/camera.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtest_mono");
  testing::InitGoogleTest(&argc, argv);

  // run the tests in this thread
  return RUN_ALL_TESTS();
}
