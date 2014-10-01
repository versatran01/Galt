#include "pcl_editor/pcl_cropper_node.hpp"

namespace pcl_editor {

using namespace pcl;

PclCropperNode::PclCropperNode(const ros::NodeHandle& nh)
    : nh_(nh),
      cloud_(new MyPointCloud),
      viewer_(new visualization::PCLVisualizer("3D Viewer")) {
  std::string pcd;
  nh_.param<std::string>("pcd", pcd, std::string());
  // Load two point cloud
  if (!(LoadPcdFile(pcd, cloud_))) {
    throw std::runtime_error("Failed to load file");
  }
  viewer_->setBackgroundColor(0, 0, 0);
  viewer_->addPointCloud<MyPoint>(cloud1_, "cloud1");
  viewer_->setPointCloudRenderingProperties(
      visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
  viewer_->addCoordinateSystem(1.0);
  viewer_->initCameraParameters();
  cfg_server_.setCallback(boost::bind(&PclMergerNode::ConfigCb, this, _1, _2));
}

}  // namespace pcl_editor
