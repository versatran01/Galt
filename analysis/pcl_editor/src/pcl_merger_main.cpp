#include "pcl_editor/pcl_merger_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "tree_merger");
  ros::NodeHandle nh("~");
  try {
    pcl_merger::PclMergerNode pcl_merger_node(nh);
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
