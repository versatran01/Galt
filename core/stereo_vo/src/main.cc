#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "stereo_vo");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
}
