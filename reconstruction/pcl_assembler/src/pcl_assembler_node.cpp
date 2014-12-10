#include "pcl_assembler/pcl_assembler_node.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

namespace pcl_assembler {

PclAssemblerNode::PclAssemblerNode(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), pcl_pc_(new PclPointCloud()) {
  if (!pnh_.getParam("fixed_frame", fixed_frame_)) {
    fixed_frame_ = "world";
    ROS_WARN("fixed framed not provided, use %s", fixed_frame_.c_str());
  }
  ROS_INFO("fixed_frame: %s", fixed_frame_.c_str());

  const std::string resolved_topic = nh_.resolveName("cloud_in");
  ROS_INFO("%s subscribing to %s", ros::this_node::getName().c_str(),
           resolved_topic.c_str());
  pc2_sub_ =
      nh_.subscribe(resolved_topic, 1, &PclAssemblerNode::PointCloud2Cb, this);
  //  pc2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_assembled", 1);
}

void PclAssemblerNode::PointCloud2Cb(
    const sensor_msgs::PointCloud2ConstPtr &pc2_msg) {
  // Convert Ros PointCloud2 message to pcl point cloud
  PclPointCloud::Ptr pcl_pc = boost::make_shared<PclPointCloud>();
  PclPointCloud::Ptr pcl_pc_filtered = boost::make_shared<PclPointCloud>();
  pcl::fromROSMsg(*pc2_msg, *pcl_pc);
  pcl::PassThrough<PointT> pass_filter;
  pass_filter.setInputCloud(pcl_pc);
  pass_filter.setFilterFieldName("z");
  pass_filter.setFilterLimits(1.0, 10.0);
  pass_filter.filter(*pcl_pc_filtered);
  ROS_INFO("%d: %d", (int)pcl_pc->size(), (int)pcl_pc_filtered->size());
  //  pcl::VoxelGrid<PointT> voxel_filter;
  //  voxel_filter.setInputCloud(pcl_pc);
  //  voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f);
  //  voxel_filter.filter(pcl_pc_filtered);
}

}  // namespace pcl_assembler

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_assembler_node");
  ros::NodeHandle nh, pnh("~");

  try {
    pcl_assembler::PclAssemblerNode pcl_assembler_node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
