#include <ros/ros.h>

#include "bluefox2/camera.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bluefox2");
  bluefox2::Camera camera(ros::NodeHandle(), ros::NodeHandle("~"));

  if (camera.ok()) {
    camera.feedImage();
  }

  return 0;
}

