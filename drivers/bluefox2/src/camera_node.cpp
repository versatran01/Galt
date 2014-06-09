#include <ros/ros.h>

#include "bluefox2/camera.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bluefox2");
  bluefox2::Camera camera(ros::NodeHandle("~"));

  // Feed image only when camera is initialized
  if (camera.ok()) {
    ROS_WARN("Start publishing...");
    ros::Rate rate(camera.fps());
    while (ros::ok()) {
      camera.feedImage();
      ros::spinOnce();
      rate.sleep();
    }
  }

  return 0;
}

