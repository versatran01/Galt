#include "pcl_assembler/pcl_assembler_node.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <eigen_conversions/eigen_msg.h>

namespace pcl_assembler {

PclAssemblerNode::PclAssemblerNode(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &pnh)
    : nh_(nh),
      pnh_(pnh),
      pcl_pc_assembled_(new PclPointCloud()),
      tf_listener_(core_) {
  pnh_.param("leaf_size", leaf_size_, 0.03);
  pnh_.param("min_z", min_z_, 0.5);
  pnh_.param("max_z", max_z_, 6.0);
  pnh_.param("assemble", do_assemble_, true);
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
  pc2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_assembled", 1);
}

// This function is full of hack, will never fix it
void PclAssemblerNode::PointCloud2Cb(
    const sensor_msgs::PointCloud2ConstPtr &pc2_msg) {
  // Listen to the latest transform from cloud to fixed_frame
  geometry_msgs::TransformStamped tf_stamped;
  if (!GetLatestTransfrom(fixed_frame_, pc2_msg->header.frame_id, tf_stamped)) {
    return;
  }
  Eigen::Affine3d affine;
  tf::transformMsgToEigen(tf_stamped.transform, affine);
  // Convert Ros PointCloud2 message to pcl point cloud
  PclPointCloud::Ptr pcl_pc = boost::make_shared<PclPointCloud>();
  PclPointCloud::Ptr pcl_pc_passed = boost::make_shared<PclPointCloud>();
  PclPointCloud::Ptr pcl_pc_filtered = boost::make_shared<PclPointCloud>();
  PclPointCloud::Ptr pcl_pc_transformed = boost::make_shared<PclPointCloud>();
  pcl::fromROSMsg(*pc2_msg, *pcl_pc);
  // Pass through filter
  pcl::PassThrough<PointT> pass_filter;
  pass_filter.setInputCloud(pcl_pc);
  pass_filter.setFilterFieldName("z");
  pass_filter.setFilterLimits(min_z_, max_z_);
  pass_filter.filter(*pcl_pc_passed);
  // First voxel grid filter
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(pcl_pc_passed);
  voxel_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxel_filter.filter(*pcl_pc_filtered);
  ROS_INFO("%d: %d: %d", (int)pcl_pc->size(), (int)pcl_pc_passed->size(),
           (int)pcl_pc_filtered->size());
  sensor_msgs::PointCloud2 pc2;
  if (do_assemble_) {
    // Transform point cloud to fixed_frame
    pcl::transformPointCloud(*pcl_pc_filtered, *pcl_pc_transformed, affine);
    pcl_pc_transformed->header.frame_id = fixed_frame_;

    if (pcl_pc_assembled_->header.frame_id.empty()) {
      // First time
      pcl_pc_assembled_ = pcl_pc_transformed;
    } else {
      // Concatenate point clouds
      *pcl_pc_assembled_ += *pcl_pc_transformed;
    }
    ROS_INFO("total: %d", (int)pcl_pc_assembled_->size());
    pcl::toROSMsg(*pcl_pc_assembled_, pc2);
  } else {
    // Only downsample, so just publish voxel filtered
    pcl::toROSMsg(*pcl_pc_filtered, pc2);
  }
  pc2_pub_.publish(pc2);
  // Publish
}

bool PclAssemblerNode::GetLatestTransfrom(
    const std::string &frame_tgt, const std::string &frame_src,
    geometry_msgs::TransformStamped &tf_stamped) const {
  try {
    tf_stamped = core_.lookupTransform(frame_tgt, frame_src, ros::Time(0));
    return true;
  }
  catch (const tf2::TransformException &e) {
    ROS_WARN_THROTTLE(1, "unable to listen to transform from %s to %s: %s",
                      frame_src.c_str(), frame_tgt.c_str(), e.what());
    return false;
  }
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
