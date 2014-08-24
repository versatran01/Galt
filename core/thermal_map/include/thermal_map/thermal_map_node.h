#ifndef GALT_THERMAL_MAP_NODE_H_
#define GALT_THERMAL_MAP_NODE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include "thermal_map/rviz_helper.h"

namespace galt {
namespace thermal_map {

class ThermalMapNode {

 public:
  ThermalMapNode(const ros::NodeHandle &nh);

 private:
  void OdomCb(const nav_msgs::OdometryConstPtr &odom_msg);
  void CameraCb(const sensor_msgs::ImageConstPtr &image_msg,
                const sensor_msgs::CameraInfoConstPtr &cinfo_msg);

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
  ros::Subscriber sub_odom_;
  ros::Publisher pub_traj_;
  TrajectoryVisualizer viz_traj_;
};
}
}
#endif  // GALT_THERMAL_MAP_NODE_H_
