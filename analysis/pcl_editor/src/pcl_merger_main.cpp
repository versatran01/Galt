#include "pcl_editor/pcl_merger_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_merger");
  ros::NodeHandle nh("~");

  try {
    pcl_editor::PclMergerNode pcl_merger_node(nh);
    pcl_merger_node.Setup();
    pcl_merger_node.Start();
  }
  catch (const std::exception& e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
