#include "pcl_editor/pcl_merger_node.hpp"

namespace pcl_merger {

PclMergerNode::PclMergerNode(const ros::NodeHandle& nh)
    : nh_(nh),
      cloud1_(new MyPointCloud),
      cloud2_(new MyPointCloud),
      viewer_(new pcl::visualization::PCLVisualizer("3D Viewer")) {
  cfg_server_.setCallback(boost::bind(&PclMergerNode::ConfigCb, this, _1, _2));
  std::string pcd1;
  std::string pcd2;
  nh_.param<std::string>("pcd1", pcd1, std::string());
  nh_.param<std::string>("pcd2", pcd2, std::string());
  nh_.param<std::string>("pcd_merged", pcd_merged_, std::string());
  // Load two point cloud
  if (!(LoadPcdFile(pcd1, cloud1_) && LoadPcdFile(pcd2, cloud2_))) {
    throw std::runtime_error("Failed to load file");
  }
  viewer_->setBackgroundColor(0, 0, 0);
  viewer_->addPointCloud<pcl::PointWithViewpoint>(cloud1_, "cloud1");
  viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
  viewer_->addCoordinateSystem(1.0);
  viewer_->initCameraParameters();
}

// void PclMergerNode::SimpleVis(const pcl::PointCloud::Ptr& cloud) {}

void PclMergerNode::ConfigCb(const MergerDynConfig& config, int level) {
  if (level < 0) {
    ROS_INFO("Initialize reconfigure server for %s",
             nh_.getNamespace().c_str());
  }
}

bool LoadPcdFile(const std::string& pcd, const MyPointCloud::Ptr& cloud) {
  if (pcl::io::loadPCDFile<pcl::PointWithViewpoint>(pcd, *cloud) == -1) {
    PCL_ERROR("Couldn't read file %s \n", pcd.c_str());
    return false;
  }
  ROS_INFO("Loaded %d points from %s", int(cloud->width * cloud->height),
           pcd.c_str());
  return true;
}

}  // namespace pcl_merge
