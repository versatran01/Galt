#include "img2pcl/img2pcl_node.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "img2pcl");
  ros::NodeHandle nh("~");

  galt::img2pcl::Img2pclNode img2pcl_node(nh);
  ros::spin();
}
