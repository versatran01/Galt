#include "rviz_helper/rviz_helper.h"

namespace rviz_helper {

TrajectoryVisualizer::TrajectoryVisualizer(const ros::NodeHandle &nh,
                                           const std::string &topic)
    : nh_{ nh }, traj_pub_(nh_.advertise<visualization_msgs::Marker>(topic, 1)),
      num_skip_(0), total_points_cnt_(0) {
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

} // namespace rviz_helper
