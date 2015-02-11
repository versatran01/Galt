//  main.cpp
#include <ros/ros.h>
#include <spectral_meter/node.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "spectral_meter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  galt::spectral_meter::Node node(nh, pnh);
  try {
    node.configure();
  } catch (std::exception& e) {
    ROS_ERROR("Error configuring: %s", e.what());
  }
  ros::spin();
  return 0;
}
