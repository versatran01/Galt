#include "pcl_editor/pcl_cropper_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_cropper");
  ros::NodeHandle nh("~");

  try {
    pcl_editor::PclCropperNode pcl_cropper_node(nh);
    pcl_cropper_node.Setup();
    pcl_cropper_node.Start();
  }
  catch (const std::exception& e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
