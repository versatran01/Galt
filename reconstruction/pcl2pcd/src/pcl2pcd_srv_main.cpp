#include "pcl2pcd/pcl2pcd_srv.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl2pcd_srv");
  ros::NodeHandle nh;
  try {
    pcl2pcd::Pcl2PcdSrv pcl2pcd_srv(nh);
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
