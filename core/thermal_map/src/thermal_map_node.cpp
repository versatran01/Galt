#include "thermal_map/thermal_map_node.h"

#include <visualization_msgs/Marker.h>

namespace galt {
namespace thermal_map {

ThermalMapNode::ThermalMapNode(const ros::NodeHandle &nh) : nh_{nh}, it_{nh} {
  image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);
  sub_camera_ = it_.subscribeCamera("color_map", 1, &ThermalMapNode::CameraCb,
                                    this, hints);

  sub_odom_ = nh_.subscribe("odometry", 1, &ThermalMapNode::OdomCb, this);
  pub_traj_ = nh_.advertise<visualization_msgs::Marker>("trajectory", 1);
  std_msgs::ColorRGBA traj_color;
  traj_color.r = 1;
  traj_color.a = 1;
  viz_traj_ = TrajectoryVisualizer(pub_traj_, traj_color, 0.2);
}

void ThermalMapNode::OdomCb(const nav_msgs::OdometryConstPtr &odom_msg) {
  viz_traj_.PublishTrajectory(odom_msg->pose.pose.position, odom_msg->header);
}

void ThermalMapNode::CameraCb(
    const sensor_msgs::ImageConstPtr &image_msg,
    const sensor_msgs::CameraInfoConstPtr &cinfo_msg) {
  ROS_INFO_THROTTLE(2, "In camera callback");
}

}  // namespace thermal_map
}  // namespace galt
