//  main.cpp
#include <ros/ros.h>
#include <spectral_meter/node.hpp>

int main(int argc, char ** argv) {
  ros::init(argc,argv,"spectral_meter");
  ros::NodeHandle pnh("~");
  galt::spectral_meter::Node node(pnh);
  node.configureTopics();
  ros::spin();
  return 0;
}
