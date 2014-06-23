#include <ros/ros.h>

#include "bluefox2/bluefox2_ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "bluefox2");
  ros::NodeHandle nh("~");

  try {
    // Here we use the wrapper class to hide all the ros stuff
    bluefox2::CameraRos camera_ros(nh);

    camera_ros.init();     // Initialize camera
    camera_ros.publish();  // Continuous publish images
  }
  catch (const std::exception &e) {
    ROS_ERROR("bluefox2: %s", e.what());
  }

  return 0;
}
