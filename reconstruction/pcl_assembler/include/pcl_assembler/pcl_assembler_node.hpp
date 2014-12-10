#ifndef PCL_ASSEMBLER_NODE_HPP_
#define PCL_ASSEMBLER_NODE_HPP_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>

namespace pcl_assembler {

class PclAssemblerNode {
 public:
  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PclPointCloud;

  PclAssemblerNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  void PointCloud2Cb(const sensor_msgs::PointCloud2ConstPtr& pc2_msg);

 private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber pc2_sub_;
  ros::Publisher pc2_pub_;
  std::string fixed_frame_;
  PclPointCloud::Ptr pcl_pc_;
};

}  // namespace pcl_assembler

#endif  // PCL_ASSEMBLER_NODE_HPP_
