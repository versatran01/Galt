#ifndef PCL_MERGER_NODE_HPP_
#define PCL_MERGER_NODE_HPP_

#include "pcl_editor/pcl_editor.hpp"

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl_editor/MergerDynConfig.h>

namespace pcl_editor {

class PclMergerNode {
 public:
  PclMergerNode(const ros::NodeHandle& nh);
  void View();
  bool Ok() const { return !viewer_->wasStopped(); }
  void SpinOnce() const { viewer_->spinOnce(100); }

 private:
  void ConfigCb(MergerDynConfig& config, int level);

  ros::NodeHandle nh_;
  dynamic_reconfigure::Server<pcl_editor::MergerDynConfig> cfg_server_;

  MyPointCloud::Ptr cloud1_, cloud2_, cloud_transformed_;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
};

}  // namespace pcl_editor

#endif  // PCL_MERGER_NODE_HPP_
