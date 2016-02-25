#ifndef PCL_ASSEMBLER_NODE_HPP_
#define PCL_ASSEMBLER_NODE_HPP_

#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/point_cloud.h>

#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl_assembler/SaveToPcdFile.h>

namespace pcl_assembler {

class PclAssemblerNode {
 public:
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointCloud<PointT> PclPointCloud;

  PclAssemblerNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  void PointCloud2Cb(const sensor_msgs::PointCloud2ConstPtr& pc2_msg);

 private:
  bool GetLatestTransfrom(const std::string& frame_tgt,
                          const std::string& frame_src,
                          geometry_msgs::TransformStamped& tf_stamped) const;
  bool SaveToPcdFile(SaveToPcdFile::Request& req, SaveToPcdFile::Response& res);

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber pc2_sub_;
  ros::Publisher pc2_pub_;
  std::string fixed_frame_;
  PclPointCloud::Ptr pcl_pc_assembled_;
  tf2::BufferCore core_;
  tf2_ros::TransformListener tf_listener_;
  ros::ServiceServer save_srv_server_;
  bool do_assemble_;
  double min_z_, max_z_;
  double leaf_size_;
};

}  // namespace pcl_assembler

#endif  // PCL_ASSEMBLER_NODE_HPP_
