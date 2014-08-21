#include <sam_estimator/sam_estimator_node.hpp>

int main(int argc, char ** argv) {
  ros::init(argc, argv, "sam_estimator");
  ros::NodeHandle nh("~");
  galt::sam_estimator::SamEstimatorNode node(nh);
  ros::spin();
  return 0;
}
