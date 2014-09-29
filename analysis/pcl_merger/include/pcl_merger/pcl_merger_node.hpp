#ifndef PCL_MERGER_NODE_HPP_
#define PCL_MERGER_NODE_HPP_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_merger/MergerDynConfig.h>

namespace pcl_merger {

class PclMergerNode {
 public:
  PclMergerNode(const ros::NodeHandle& nh);

 private:
  void ConfigCb(const pcl_merger::MergerDynConfig& config, int level);

  ros::NodeHandle nh_;
  dynamic_reconfigure::Server<pcl_merger::MergerDynConfig> cfg_server_;
};

}  // namespace pcl_merger

#endif  // PCL_MERGER_NODE_HPP_
