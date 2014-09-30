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
      tf_listener_(core_),
      save_srv_server_(
          nh_.advertiseService("save_to_pcd", &Pcl2PcdRviz::SaveToPcd, this)) {
  pnh_.param<int>("queue_size", queue_size_, 1000);
}

bool Pcl2PcdRviz::SaveToPcd(SaveToPcd::Request &req, SaveToPcd::Response &res) {
  if (clouds_.empty()) {
    res.success = false;
    ROS_INFO("No clouds to save");
    return true;
  }
  ROS_INFO("Saveing %d clouds to %s", (int)clouds_.size(),
           req.filename.c_str());
  for (size_t i = 0; i < clouds_.size(); ++i) {
    std::string file = req.filename + "_" + std::to_string(i) + ".pcd";
    try {
      pcl::io::savePCDFile(file, *(clouds_[i]));
      ROS_INFO("Saved cloud %d to: %s", (int)i, file.c_str());
    }
    catch (const std::exception &e) {
      ROS_ERROR("%s: %s", nh_.getNamespace().c_str(), e.what());
    }
  }
  res.success = true;
  return true;
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
      ROS_INFO("Add collection %d of %d points to clouds", (int)clouds_.size(),
               (int)cloud_->size());
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
    cloud_->height = 1;
    cloud_->is_dense = false;
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

  // Build cloud_xyz into a big cloud
  BuildCloud(cloud_xyz, tf_stamped.transform.translation);
}

void Pcl2PcdRviz::BuildCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                             const geometry_msgs::Vector3 &pos) {
  cloud_->reserve(cloud_->size() + cloud.size());
  cloud_->width += cloud.width;
  for (const pcl::PointXYZ &p : cloud.points) {
    cloud_->points.emplace_back(p.x, p.y, p.z, pos.x, pos.y, pos.z);
  }
}

}  // namespace pcl2pcd
