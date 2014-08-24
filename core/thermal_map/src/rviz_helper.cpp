#include "thermal_map/rviz_helper.h"

namespace galt {
namespace thermal_map {

TrajectoryVisualizer::TrajectoryVisualizer() {
  markers_.action = visualization_msgs::Marker::ADD;
  markers_.type = visualization_msgs::Marker::LINE_STRIP;
  markers_.lifetime = ros::Duration();
  markers_.pose.orientation.w = 1.0;
}

TrajectoryVisualizer::TrajectoryVisualizer(const ros::Publisher &pub,
                                           const std_msgs::ColorRGBA &color,
                                           const double scale)
    : TrajectoryVisualizer() {
  pub_ = pub;
  markers_.color = color;
  markers_.scale.x = scale;
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
  pub_.publish(markers_);
}

}  // namespace thermal_map
}  // namespace galt
