#ifndef PCL2PCD_RVIZ_HPP_
#define PCL2PCD_RVIZ_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <pcl2pcd/SaveToPcd.h>

namespace pcl2pcd {

class Pcl2PcdRviz {
 public:
  Pcl2PcdRviz(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

 private:
  typedef pcl::PointCloud<pcl::PointWithViewpoint> MyPointCloud;

  void GoalCb(const geometry_msgs::PoseStampedConstPtr&);
  void PoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
  void CloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void BuildCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                  const geometry_msgs::Vector3& pos);
  bool SaveToPcd(pcl2pcd::SaveToPcd::Request& req,
                 pcl2pcd::SaveToPcd::Response& res);

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_goal_;
  ros::Subscriber sub_pose_;
  ros::Subscriber sub_cloud_;
  MyPointCloud::Ptr cloud_;
  std::vector<MyPointCloud::Ptr> clouds_;
  int queue_size_;
  tf2::BufferCore core_;
  tf2_ros::TransformListener tf_listener_;
  ros::ServiceServer save_srv_server_;
};

}  // namespace pcl2pcd

#endif  // PCL2PCD_RVIZ_HPP_
