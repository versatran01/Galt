#include "pcl2pcd/pcl2pcd_node.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace pcl2pcd {

using namespace pcl;

bool Pcl2PcdNode::SaveToPcd(SaveToPcd::Request &req, SaveToPcd::Response &res) {
  const ros::Time curr_stamp = ros::Time::now();
  ROS_INFO("Assembling cloud from %f to %f", prev_stamp_.toSec(),
           curr_stamp.toSec());

  laser_assembler::AssembleScans2 srv;
  srv.request.end = curr_stamp;
  srv.request.begin = prev_stamp_;

  if (!srv_client_.call(srv)) {
    ROS_WARN_THROTTLE(1, "%s: Sercie call failed", nh_.getNamespace().c_str());
    return false;
  }

  if (srv.response.cloud.data.empty()) {
    ROS_WARN_THROTTLE(1, "%s: Empty cloud", nh_.getNamespace().c_str());
    return false;
  }

  ROS_INFO("Got cloud with %ld points", srv.response.cloud.data.size());

  // Get point cloud assembled from assembler and save to a file
  const sensor_msgs::PointCloud2 &cloud_ros_w = srv.response.cloud;
  PointCloud<PointXYZ> cloud_pcl_w;
  pcl::fromROSMsg(cloud_ros_w, cloud_pcl_w);
  try {
    pcl::io::savePCDFile(req.filename, cloud_pcl_w);
    ROS_INFO("Cloud saved to %s", req.filename.c_str());
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", nh_.getNamespace().c_str(), e.what());
    return false;
  }

  prev_stamp_ = curr_stamp;
  res.success = true;
  return true;
}

}  // namespace pcl2pcd