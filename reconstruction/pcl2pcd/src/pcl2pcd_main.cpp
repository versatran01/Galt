#include "pcl2pcd/pcl2pcd_node.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl2pcd");
  ros::NodeHandle nh;
  try {
    pcl2pcd::Pcl2PcdNode pcl2pcd_node(nh);
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str());
  }
}
