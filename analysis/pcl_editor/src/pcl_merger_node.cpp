#include "pcl_editor/pcl_merger_node.hpp"
#include <eigen3/Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <tf2/LinearMath/Quaternion.h>

namespace pcl_editor {

using namespace pcl;

void PclMergerNode::InitializeViewer() {
  super::InitializeViewer();
  // Add cloud1
  viewer_->addPointCloud<MyPoint>(cloud1_, id1_);
  viewer_->setPointCloudRenderingProperties(
      visualization::PCL_VISUALIZER_POINT_SIZE, 1, id2_);
  // Add cloud2 with a different color
  MyColorHandler single_color(cloud2_, 0, 255, 0);
  viewer_->addPointCloud<MyPoint>(cloud2_, single_color, id2_);
}

void PclMergerNode::EditPointCloud() {
  cloud_transformed_.reset(new MyPointCloud);
  // Get the transform to transform cloud2
  Eigen::Vector3f t(config_.x, config_.y, config_.z);
  tf2::Quaternion tf_q(config_.yaw, config_.pitch, config_.roll);
  Eigen::Quaternionf q(tf_q.getW(), tf_q.getX(), tf_q.getY(), tf_q.getZ());
  transformPointCloud(*cloud2_, *cloud_transformed_, t, q);
  // Visualize the  transformed cloud
  MyColorHandler my_color(cloud_transformed_, 0, 255, 0);
  viewer_->updatePointCloud<MyPoint>(cloud_transformed_, my_color, id2_);
}

void PclMergerNode::SavePointCloud() {
  if (cloud_transformed_->empty()) return;
  // Merge two clouds
  MyPointCloud cloud_merged;
  cloud_merged = *cloud1_;
  cloud_merged += *cloud_transformed_;
  ROS_INFO("Merging cloud, %zu + %zu = %zu", cloud1_->size(),
           cloud_transformed_->size(), cloud_merged.size());
  try {
    pcl::io::savePCDFile(config_.pcd_merged, cloud_merged);
    ROS_INFO("Saved merged cloud to: %s", config_.pcd_merged.c_str());
  }
  catch (const std::exception& e) {
    ROS_ERROR("%s: %s", nh_.getNamespace().c_str(), e.what());
  }
}

}  // namespace pcl_editor
