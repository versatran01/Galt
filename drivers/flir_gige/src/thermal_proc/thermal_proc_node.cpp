#include "flir_gige/thermal_proc/thermal_proc.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "thermal_proc");

  ros::NodeHandle nh("~");

  try {
    flir_gige::ThermalProc thermal_proc(nh);
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR_STREAM("flir_gige: thermal_proc: " << e.what());
  }
}
