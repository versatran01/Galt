#include "iris_tf/rviz_helper.h"

namespace galt {
namespace iris_tf {

TrajectoryVisualizer::TrajectoryVisualizer(const ros::NodeHandle &nh)
    : nh_{nh}, traj_pub_(nh_.advertise<visualization_msgs::Marker>("traj", 1)) {
  markers_.scale.x = 0.05;
  markers_.action = visualization_msgs::Marker::ADD;
  markers_.type = visualization_msgs::Marker::LINE_STRIP;
  markers_.lifetime = ros::Duration();
  markers_.pose.orientation.w = 1.0;
  markers_.color.r = 1;
  markers_.color.a = 1;
}

void TrajectoryVisualizer::PublishTrajectory(const geometry_msgs::Point &point,
                                             const std_msgs::Header &header) {
  PublishTrajectory(point, header.frame_id, header.stamp);
}

void TrajectoryVisualizer::PublishTrajectory(const geometry_msgs::Point &point,
                                             const std::string &frame_id,
                                             const ros::Time &time) {
  markers_.points.push_back(point);
  markers_.header.frame_id = frame_id;
  markers_.header.stamp = time;
  traj_pub_.publish(markers_);
}

}  // namespace iris_tf
}  // namespace galt
