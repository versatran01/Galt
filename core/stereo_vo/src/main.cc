#include <ros/ros.h>

#include "stereo_vo/stereo_vo_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "stereo_vo");
  ros::NodeHandle nh("~");

  try {  
    galt::stereo_vo::StereoVoNode stereo_vo_node(nh);
    ros::spin();
  } catch (std::exception& e) {
    ROS_ERROR("Fatal error: %s", e.what());
    return -1;
  }
  return 0;
}
