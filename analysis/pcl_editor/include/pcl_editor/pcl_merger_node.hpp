#ifndef PCL_MERGER_NODE_HPP_
#define PCL_MERGER_NODE_HPP_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_merger/MergerDynConfig.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace pcl_merger {

typedef pcl::PointCloud<pcl::PointWithViewpoint> MyPointCloud;

class PclMergerNode {
 public:
  PclMergerNode(const ros::NodeHandle& nh);

 private:
  void ConfigCb(const pcl_merger::MergerDynConfig& config, int level);
  //  void SimpleVis(const MyPointCloud::Ptr& cloud);

  ros::NodeHandle nh_;
  dynamic_reconfigure::Server<pcl_merger::MergerDynConfig> cfg_server_;
  MyPointCloud::Ptr cloud1_, cloud2_;
  std::string pcd_merged_;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
};

bool LoadPcdFile(const std::string& pcd, const MyPointCloud::Ptr& cloud);

}  // namespace pcl_merger

#endif  // PCL_MERGER_NODE_HPP_
