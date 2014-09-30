#include "pcl2pcd/pcl2pcd_rviz.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl2pcd_rviz");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  try {
    pcl2pcd::Pcl2PcdRviz pcl2pcd_rviz(nh, pnh);
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
