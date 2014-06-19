#include <string>
#include <ros/ros.h>
#include "bluefox2/bluefox2.h"

#define btoa(x) ((x) ? "true" : "false")

using std::string;

bluefox2::mv_params_t ReadParams(ros::NodeHandle nh);

int main(int argc, char **argv) {
  ros::init(argc, argv, "bluefox2");
  ros::NodeHandle nh("~");

  // Get camera serial number
  string serial;
  nh.param<string>("serial", serial, "");

  // Get node information
  string topic("image_raw");
  string node = string("cam_") + serial;

  // Get settings from launch file
  bluefox2::mv_params_t mv_params;
  mv_params = ReadParams(nh);

  // Get other ros related settings
  string calibration_url;
  string frame_id;
  nh.param<string>("calibration_url", calibration_url, "");
  nh.param<string>("frame_id", frame_id, "");

  // Instantiate and initialize camera
  bluefox2::Camera camera(serial, mv_params);
  ROS_INFO("Device count: %d", camera.device_count());
  camera.init(true);  // true - print settings

  // Set up camera publisher and check camera calibration

  // Publish images
  if (camera.ok()) {
    ros::Rate loop_rate(camera.fps());
    ROS_INFO("Publishing image topic to /%s/%s", node.c_str(), topic.c_str());

    int seq = 0;
    while (ros::ok()) {
      // Grab image from camera
      ROS_INFO("%d", seq++);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  return 0;
}

bluefox2::mv_params_t ReadParams(ros::NodeHandle nh) {
  bluefox2::mv_params_t mv_params;

  nh.param<bool>("color", mv_params.color, false);
  nh.param<bool>("hdr", mv_params.hdr, false);
  nh.param<bool>("inverted", mv_params.inverted, false);
  nh.param<bool>("binning", mv_params.binning, false);
  nh.param<bool>("auto_exposure", mv_params.auto_exposure, false);
  nh.param<int>("exposure_time_us", mv_params.exposure_time_us, 10000);
  nh.param<int>("height", mv_params.height, 480);
  nh.param<int>("width", mv_params.width, 752);
  nh.param<int>("fps", mv_params.fps, 20);
  nh.param<double>("gain", mv_params.gain, 0.0);
  nh.param<string>("mode", mv_params.mode, "standalone");
  nh.param<string>("while_balance", mv_params.white_balance, "default");

  return mv_params;
}
