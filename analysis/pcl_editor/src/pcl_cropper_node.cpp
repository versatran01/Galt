#include "pcl_editor/pcl_cropper_node.hpp"

namespace pcl_editor {

using namespace pcl;

void PclCropperNode::FilterByXYZ(MyPointCloud &cloud_filtered) {
  // Filter by x
  MyPointCloud::Ptr filter_x(new MyPointCloud);
  pass_x_.setInputCloud(cloud_);
  pass_x_.setFilterLimits(config_.x - config_.d_x, config_.x + config_.d_x);
  pass_x_.filter(*filter_x);

  // Filter by y
  MyPointCloud::Ptr filter_y(new MyPointCloud);
  pass_y_.setInputCloud(filter_x);
  pass_y_.setFilterLimits(config_.y - config_.d_y, config_.y + config_.d_y);
  pass_y_.filter(*filter_y);

  // Filter by z
  pass_z_.setInputCloud(filter_y);
  pass_z_.setFilterLimits(config_.z - config_.d_z, config_.z + config_.d_z);
  pass_z_.filter(cloud_filtered);
}

void PclCropperNode::InitializeViewer() {
  super::InitializeViewer();
  // Add cloud
  viewer_->addPointCloud<MyPoint>(cloud_, id_);
  viewer_->setPointCloudRenderingProperties(
      visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_);
  // Determining initial filter limits
  InitializeConfig(*cloud_, config_);
  // Add cloud cropped with no cropping
  FilterByXYZ(*cloud_cropped_);
  MyColorHandler my_color(cloud_cropped_, 0, 255, 0);
  viewer_->addPointCloud<MyPoint>(cloud_cropped_, my_color, id_cropped_);
  viewer_->setPointCloudRenderingProperties(
      visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_cropped_);
}

void PclCropperNode::EditPointCloud() {
  // Crop original point cloud
  cloud_cropped_.reset(new MyPointCloud);
  if (config_.reset) {
    ROS_INFO("Reseting crop area");
    InitializeConfig(*cloud_, config_);
    config_.reset = false;
  }
  FilterByXYZ(*cloud_cropped_);
  // Visualize the cropped cloud
  MyColorHandler my_color(cloud_cropped_, 0, 255, 0);
  viewer_->updatePointCloud<MyPoint>(cloud_cropped_, my_color, id_cropped_);
}

void PclCropperNode::SavePointCloud() {
  if (cloud_cropped_->empty()) return;
  ROS_INFO("Cropped cloud, %zu", cloud_cropped_->size());
  SaveToPcd(config_.pcd_cropped, *cloud_cropped_);
}

void InitializeConfig(const MyPointCloud &cloud, CropperDynConfig &config) {
  MyPoint min_pt, max_pt;
  pcl::getMinMax3D(cloud, min_pt, max_pt);
  config.x = (min_pt.x + max_pt.x) / 2;
  config.y = (min_pt.y + max_pt.y) / 2;
  config.z = (min_pt.z + max_pt.z) / 2;
  config.d_x = max_pt.x - config.x;
  config.d_y = max_pt.y - config.y;
  config.d_z = max_pt.z - config.z;
}

}  // namespace pcl_editor
