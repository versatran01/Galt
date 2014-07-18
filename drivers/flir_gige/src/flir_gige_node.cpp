#include <stdexcept>

#include <ros/ros.h>

#include "flir_gige/flir_gige.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "flir_node");

  ros::NodeHandle nh("~");

  try {
    flir_gige::FlirGige flir_gige(nh);
    flir_gige.Run();
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR_STREAM(e.what());
  }

  return 0;
}
