#include "img2pcl/img2pcl_node.h"

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "img2pcl");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    galt::img2pcl::Img2pclNode img2pcl_node(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
