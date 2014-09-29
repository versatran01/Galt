#ifndef PCL_MERGER_NODE_HPP_
#define PCL_MERGER_NODE_HPP_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_merger/MergerDynConfig.h>
#include <pcl_ros/point_cloud.h>

namespace pcl_merger {

class PclMergerNode {
 public:
  PclMergerNode(const ros::NodeHandle& nh);

 private:
  void ConfigCb(const pcl_merger::MergerDynConfig& config, int level);

  ros::NodeHandle nh_;
  dynamic_reconfigure::Server<pcl_merger::MergerDynConfig> cfg_server_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;
};

}  // namespace pcl_merger

#endif  // PCL_MERGER_NODE_HPP_
