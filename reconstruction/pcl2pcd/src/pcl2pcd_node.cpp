#include "pcl2pcd/pcl2pcd_node.hpp"

namespace pcl2pcd {

bool Pcl2PcdNode::SaveToPcd(SaveToPcd::Request &req, SaveToPcd::Response &res) {
  ROS_INFO("%s", req.filename.c_str());
  res.success = true;
  return res.success;
}

}  // namespace pcl2pcd
