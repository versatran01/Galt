#include "pcl_assembler/pcl_filter_node.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

namespace pcl_assembler {

PclFilterNode::PclFilterNode(const ros::NodeHandle &nh,
                             const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
  pnh_.param("min_z", min_z_, 0.5);
  pnh_.param("max_z", max_z_, 6.0);
  pnh_.param("leaf_size", leaf_size_, 0.01);

  const std::string resolved_topic = nh_.resolveName("cloud_in");
  ROS_INFO("%s subscribing to %s", ros::this_node::getName().c_str(),
           resolved_topic.c_str());
  pc2_sub_ =
      nh_.subscribe(resolved_topic, 1, &PclFilterNode::PointCloud2Cb, this);
  pc2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 1);
}

void PclFilterNode::PointCloud2Cb(
    const sensor_msgs::PointCloud2ConstPtr &pc2_msg) {
  // Convert to pcl type
  PclPointCloud::Ptr pcl_pc = boost::make_shared<PclPointCloud>();
  pcl::fromROSMsg(*pc2_msg, *pcl_pc);

  // Pass through filter
  PclPointCloud::Ptr pcl_pc_passed = boost::make_shared<PclPointCloud>();
  pcl::PassThrough<PointT> pass_filter;
  pass_filter.setInputCloud(pcl_pc);
  pass_filter.setFilterFieldName("z");
  pass_filter.setFilterLimits(min_z_, max_z_);
  pass_filter.filter(*pcl_pc_passed);

  //  pc2_pub_.publish(*pcl_pc_passed);
  // First voxel grid filter
  PclPointCloud::Ptr pcl_pc_filtered = boost::make_shared<PclPointCloud>();
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(pcl_pc_passed);
  voxel_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxel_filter.filter(*pcl_pc_filtered);
  pc2_pub_.publish(*pcl_pc_filtered);
}

}  // namespace pcl_assembler

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_filter_node");
  ros::NodeHandle nh, pnh("~");

  try {
    pcl_assembler::PclFilterNode pcl_filter_node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
