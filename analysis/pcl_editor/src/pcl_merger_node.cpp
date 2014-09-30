#include "pcl_editor/pcl_merger_node.hpp"

namespace pcl_merger {

PclMergerNode::PclMergerNode(const ros::NodeHandle& nh) : nh_(nh) {
  cfg_server_.setCallback(boost::bind(&PclMergerNode::ConfigCb, this, _1, _2));
}

void PclMergerNode::ConfigCb(const MergerDynConfig& config, int level) {
  if (level < 0) {
    ROS_INFO("Initialize reconfigure server for %s",
             nh_.getNamespace().c_str());
  }
}

}  // namespace pcl_merge
