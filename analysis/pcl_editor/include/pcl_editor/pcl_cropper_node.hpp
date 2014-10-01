#ifndef PCL_CROPPER_NODE_HPP_
#define PCL_CROPPER_NODE_HPP_

#include "pcl_editor/pcl_editor.hpp"

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_editor/CropperDynConfig.h>

namespace pcl_editor {

class PclCropperNode {
 public:
  PclCropperNode(const ros::NodeHandle& nh);

  bool Ok() const { return !viewer_->wasStopped(); }
  void SpinOnce() const { viewer_->spinOnce(100); }

 private:
  void ConfigCb(CropperDynConfig& config, int level);

  ros::NodeHandle nh_;
  dynamic_reconfigure::Server<pcl_editor::CropperDynConfig> cfg_server_;

  MyPointCloud::Ptr cloud_, cloud_cropped_;
  pcl::visualization::PCLVisualizer::Ptr viewer_;
};

}  // namespace pcl_editor

#endif  // PCL_CROPPER_NODE_HPP_
