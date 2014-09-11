#include <ros/ros.h>

#include "stereo_vo/stereo_vo_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "stereo_vo");
  ros::NodeHandle nh("~");

  try {
    galt::stereo_vo::StereoVoNode stereo_vo_node(nh);
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
