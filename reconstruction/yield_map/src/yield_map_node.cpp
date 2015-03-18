#include "yield_map/yield_map_node.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/contrib/contrib.hpp>

namespace yield_map {

YieldMapNode::YieldMapNode(const ros::NodeHandle& pnh) : pnh_(pnh) {
  int queue_size;
  pnh_.param("queue_size", queue_size, 10);
  pnh_.param("min_counts", min_counts_, 0.0);
  pnh_.param("max_counts", max_counts_, 6.0);
  pnh_.param("leaf_size", leaf_size_, 0.03);
  pnh_.param("min_z", min_z_, -1.0);
  pnh_.param("max_z", max_z_, 10.0);

  pcl_pc_assembled_ = boost::make_shared<PclPointCloudColor>();

  sub_cloud2_ =
      pnh_.subscribe("cloud2", queue_size, &YieldMapNode::Cloud2Cb, this);
  sub_illuminance_ = pnh_.subscribe("illuminance", queue_size,
                                    &YieldMapNode::IlluminanceCb, this);
  pub_cloud2_ =
      pnh_.advertise<sensor_msgs::PointCloud2>("yield_cloud2", queue_size);
  save_srv_server_ =
      pnh_.advertiseService("save_to_pcd", &YieldMapNode::SaveToPcd, this);
}

void YieldMapNode::IlluminanceCb(
    const sensor_msgs::IlluminanceConstPtr& illuminance_msg) {
  // Save this counts
  counts_ = illuminance_msg->illuminance * 0.2 + counts_ * 0.8;
}

void YieldMapNode::Cloud2Cb(
    const sensor_msgs::PointCloud2ConstPtr& cloud2_msg) {
  // Convert Rso PointCloud2 message to pcl PointCloud
  PclPointCloud::Ptr pcl_pc = boost::make_shared<PclPointCloud>();
  PclPointCloud::Ptr pcl_pc_passed = boost::make_shared<PclPointCloud>();
  PclPointCloud::Ptr pcl_pc_filtered = boost::make_shared<PclPointCloud>();
  pcl::fromROSMsg(*cloud2_msg, *pcl_pc);
  // Pass through filter
  pcl::PassThrough<pcl::PointXYZ> pass_filter;
  pass_filter.setInputCloud(pcl_pc);
  pass_filter.setFilterFieldName("z");
  pass_filter.setFilterLimits(min_z_, max_z_);
  pass_filter.filter(*pcl_pc_passed);
  // Voxel grid filter
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(pcl_pc_passed);
  voxel_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxel_filter.filter(*pcl_pc_filtered);
  // Convert scale to jet
  double clamped_counts = std::max(std::min(counts_, max_counts_), min_counts_);
  int gray_value = static_cast<int>(clamped_counts / max_counts_ * 255);
  cv::Mat gray(1, 1, CV_8UC1, gray_value);
  cv::Mat jet;
  cv::applyColorMap(gray, jet, cv::COLORMAP_JET);
  uchar* p = jet.ptr<uchar>(0);
  // Copy cloud to color clound
  PclPointCloudColor pcl_pc_color;
  pcl::copyPointCloud(*pcl_pc_filtered, pcl_pc_color);
  for (size_t i = 0; i < pcl_pc_filtered->size(); ++i) {
    pcl_pc_color.points[i].r = p[2];
    pcl_pc_color.points[i].g = p[1];
    pcl_pc_color.points[i].b = p[0];
  }
  // Assemble
  if (pcl_pc_assembled_->header.frame_id.empty()) {
    *pcl_pc_assembled_ = pcl_pc_color;
  } else {
    *pcl_pc_assembled_ += pcl_pc_color;
    ROS_INFO_THROTTLE(5, "Number of points: %d",
                      (int)pcl_pc_assembled_->size());
  }

  pub_cloud2_.publish(pcl_pc_color);

  //  ROS_INFO("color: %d, %d, %d", (int)p[0], (int)p[1], (int)p[2]);
  //  ROS_INFO("size: %d -> %d -> %d", (int)pcl_pc->size(),
  //           (int)pcl_pc_passed->size(), (int)pcl_pc_filtered->size());
}

bool YieldMapNode::SaveToPcd(yield_map::SaveToPcd::Request& req,
                             yield_map::SaveToPcd::Response& res) {
  // Another voxel filter
  PclPointCloudColor pcl_pc_filtered;
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
  voxel_filter.setInputCloud(pcl_pc_assembled_);
  voxel_filter.setLeafSize(0.15, 0.15, 0.15);
  voxel_filter.filter(pcl_pc_filtered);
  ROS_INFO("Voxel Filter: %d - > %d", (int)pcl_pc_assembled_->size(),
           (int)pcl_pc_filtered.size());
  try {
    pcl::io::savePCDFile(req.filename, pcl_pc_filtered);
    ROS_INFO("Cloud saved to %s", req.filename.c_str());
    res.success = true;
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh_.getNamespace().c_str(), e.what());
    res.success = false;
  }

  return res.success;
}

}  // namespace yield_map

int main(int argc, char** argv) {
  ros::init(argc, argv, "yield_map");
  ros::NodeHandle pnh("~");

  try {
    yield_map::YieldMapNode yield_map_node(pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
