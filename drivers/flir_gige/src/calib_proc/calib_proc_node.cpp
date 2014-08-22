#include "flir_gige/calib_proc/calib_proc.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "calib_proc_node");
  ros::NodeHandle cnh;
  ros::NodeHandle pnh("~");

  try {
    flir_gige::CalibProc calib_proc(cnh, pnh);
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR_STREAM("flir_gige: " << e.what());
    return -1;
  }

  return 0;
}
