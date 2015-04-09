#ifndef YIELD_MAP_H_
#define YIELD_MAP_H_

#include <ros/ros.h>
#include <sensor_msgs/Illuminance.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <yield_map/SaveToPcd.h>

namespace yield_map {

class YieldMapNode {
 public:
  typedef pcl::PointCloud<pcl::PointXYZ> PclPointCloud;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PclPointCloudColor;
  explicit YieldMapNode(const ros::NodeHandle& pnh);

  void IlluminanceCb(const sensor_msgs::IlluminanceConstPtr& illuminance_msg);
  void Cloud2Cb(const sensor_msgs::PointCloud2ConstPtr& cloud2_msg);
  bool SaveToPcd(yield_map::SaveToPcd::Request& req,
                 yield_map::SaveToPcd::Response& res);

 private:
  ros::NodeHandle pnh_;
  ros::Subscriber sub_illuminance_;
  ros::Subscriber sub_cloud2_;
  ros::Publisher pub_cloud2_;
  double counts_;
  double min_counts_, max_counts_;
  double min_z_, max_z_;
  double leaf_size_;
  PclPointCloudColor::Ptr pcl_pc_assembled_;
  ros::ServiceServer save_srv_server_;
};

}  // namespace yield_map

#endif  // YIELD_MAP_H_
