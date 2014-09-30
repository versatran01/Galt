#include "pcl_editor/pcl_merger_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "tree_merger");
  ros::NodeHandle nh("~");
  ros::Rate rate(10);
  try {
    pcl_editor::PclMergerNode pcl_merger_node(nh);
    while (ros::ok() && pcl_merger_node.Ok()) {
      ros::spinOnce();
      pcl_merger_node.SpinOnce();
      rate.sleep();
    }
  }
  catch (const std::exception& e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
