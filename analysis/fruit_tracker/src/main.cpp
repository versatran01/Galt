#include <fruit_tracker/node.hpp>

int main(int argc, char ** argv) {
  ros::init(argc,argv,"fruit_tracker");
  ros::NodeHandle pnh("~");
  galt::fruit_tracker::Node node(pnh);
  ros::spin();
  return 0;
}
