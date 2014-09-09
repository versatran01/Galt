#include "rviz_helper/rviz_helper.h"

namespace rviz_helper {

TrajectoryVisualizer::TrajectoryVisualizer(const ros::NodeHandle &nh,
                                           const std::string &topic)
    : nh_{ nh }, traj_pub_(nh_.advertise<visualization_msgs::Marker>(topic, 1)),
      num_skip_(1), total_points_cnt_(0) {
  markers_.pose.orientation.w = 1.0;
  set_colorRGB(colors::RED);
  set_scale(0.05);
  markers_.action = visualization_msgs::Marker::ADD;
  markers_.type = visualization_msgs::Marker::LINE_STRIP;
  markers_.lifetime = ros::Duration();
}

void TrajectoryVisualizer::PublishTrajectory(const geometry_msgs::Point &point,
                                             const std_msgs::Header &header) {
  PublishTrajectory(point, header.frame_id, header.stamp);
}

void TrajectoryVisualizer::PublishTrajectory(const geometry_msgs::Point &point,
                                             const std::string &frame_id,
                                             const ros::Time &time) {
  ++total_points_cnt_;
  if (total_points_cnt_ % num_skip_) {
    return;
  }
  markers_.points.push_back(point);
  markers_.header.frame_id = frame_id;
  markers_.header.stamp = time;
  traj_pub_.publish(markers_);
}

CovarianceVisualizer::CovarianceVisualizer(const ros::NodeHandle& nh,
                     const std::string &topic) :
  nh_(nh) {
  cov_pub_ = nh_.advertise<visualization_msgs::Marker>(topic, 1);
  //  default to a beige colour
  set_colorRGB({1, 0.9255, 0.5});
  
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.type = visualization_msgs::Marker::SPHERE;
  marker_.lifetime = ros::Duration(); //  last forever
}

void
CovarianceVisualizer::PublishCovariance(const nav_msgs::Odometry& odometry) {
  marker_.header = odometry.header;
  marker_.pose.position = odometry.pose.pose.position;
  marker_.pose.orientation.w = 1;
  marker_.scale.x = odometry.pose.covariance[(0 * 6) + 0];
  marker_.scale.y = odometry.pose.covariance[(1 * 6) + 1];
  marker_.scale.z = odometry.pose.covariance[(2 * 6) + 2];
  cov_pub_.publish(marker_);
}

} // namespace rviz_helper
