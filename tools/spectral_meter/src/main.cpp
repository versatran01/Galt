//  main.cpp
#include <ros/ros.h>
#include <spectral_meter/node.hpp>

int main(int argc, char ** argv) {
  ros::init(argc,argv,"spectral_meter");
  ros::NodeHandle pnh("~");
  galt::spetral_meter::Node node(pnh);
  ros::spin();
  return 0;
}
