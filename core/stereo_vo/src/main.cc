#include <ros/ros.h>

#include "stereo_vo/stereo_vo_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "stereo_vo");
  ros::NodeHandle nh("~");

  galt::stereo_vo::StereoVoNode stereo_vo_node(nh);

  ros::spin();
}
