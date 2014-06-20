#include <ros/ros.h>
#include <gtest/gtest.h>
#include "bluefox2/bluefox2.h"

bluefox2::mv_params_t ReadParams(ros::NodeHandle nh) {
  bluefox2::mv_params_t mv_params;

  nh.param<bool>("color", mv_params.color, false);
  nh.param<bool>("hdr", mv_params.hdr, false);
  nh.param<bool>("inverted", mv_params.inverted, false);
  nh.param<bool>("binning", mv_params.binning, false);
  nh.param<bool>("auto_expose", mv_params.auto_expose, false);
  nh.param<int>("expose_us", mv_params.expose_us, 10000);
  nh.param<int>("height", mv_params.height, 480);
  nh.param<int>("width", mv_params.width, 752);
  nh.param<int>("fps", mv_params.fps, 20);
  nh.param<double>("gain", mv_params.gain, 0.0);
  nh.param<string>("mode", mv_params.mode, "standalone");
  nh.param<string>("white_balance", mv_params.white_balance, "default");

  return mv_params;
}

TEST(CameraTest, construstor) {
  ros::NodeHandle nh("~");
  string serial;
  nh.param<string>("serial", serial, "");
  bluefox2::mv_params_t mv_params = ReadParams(nh);
  bluefox2::Camera camera(serial, mv_params);
  camera.init(false);
  EXPECT_EQ(10.0, camera.fps());
  EXPECT_TRUE(camera.ok());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtest_mono");
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
