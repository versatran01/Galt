#include "pcl2pcd/pcl2pcd_rviz.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <tf2/exceptions.h>

namespace pcl2pcd {

Pcl2PcdRviz::Pcl2PcdRviz(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh),
      pnh_(pnh),
      sub_goal_(nh_.subscribe("/move_base_simple/goal", 1, &Pcl2PcdRviz::GoalCb,
                              this)),
      sub_pose_(nh_.subscribe("/initialpose", 1, &Pcl2PcdRviz::PoseCb, this)),
      tf_listener_(core_) {
  pnh_.param<int>("queue_size", queue_size_, 1000);
}

void Pcl2PcdRviz::GoalCb(
    const geometry_msgs::PoseStampedConstPtr &pose_stamped) {
  // This indicates when to start to assemble clouds
  if (sub_cloud_) {
    ROS_INFO("Stop recording point cloud");
    sub_cloud_.shutdown();
    // Put into clouds if cloud is non-empty
    if (!cloud_->empty()) {
      clouds_.push_back(cloud_);
      ROS_INFO_STREAM("Add collection " << clouds_.size() << " to clouds");
    }
  }
}

void Pcl2PcdRviz::PoseCb(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_cov_stamped) {
  // This indicates when to stop to assemble clouds
  if (!sub_cloud_) {
    sub_cloud_ = nh_.subscribe("cloud_filtered", queue_size_,
                               &Pcl2PcdRviz::CloudCb, this);
    ROS_INFO_STREAM("Start recording point cloud " << (clouds_.size() + 1));
    cloud_.reset(new MyPointCloud);
  }
}

void Pcl2PcdRviz::CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  if (cloud_msg->data.empty()) {
    ROS_WARN_THROTTLE(1, "Empty cloud");
    return;
  }

  // Get transformation
  geometry_msgs::TransformStamped tf_stamped;
  try {
    tf_stamped = core_.lookupTransform(cloud_msg->header.frame_id, "laser",
                                       ros::Time(0));
  }
  catch (const tf2::TransformException &e) {
    ROS_WARN_THROTTLE(1, "%s", e.what());
    return;
  }

  // Convert PointCloud2 to pcl PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  pcl::fromROSMsg(*cloud_msg, cloud_xyz);
}

}  // namespace pcl2pcd
