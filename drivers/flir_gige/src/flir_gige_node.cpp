#include <stdexcept>

#include <ros/ros.h>

#include "flir_gige/flir_gige.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "flir_node");

  ros::NodeHandle nh("~");
  double fps = 0;
  nh.param<double>("fps", fps, 20);

  try {
    flir_gige::FlirNode flir_node(nh, fps);
    flir_node.Init();
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR_STREAM(e.what());
  }

  return 0;
}
