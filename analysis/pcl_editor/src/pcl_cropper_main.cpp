#include "pcl_editor/pcl_cropper_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_cropper");
  ros::NodeHandle nh("~");
  ros::Rate rate(10);

  try {
    pcl_editor::PclCropperNode pcl_cropper_node(nh);
    while (ros::ok() && pcl_cropper_node.Ok()) {
      ros::spinOnce();
      pcl_cropper_node.SpinOnce();
      rate.sleep();
    }
  }
  catch (const std::exception& e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
