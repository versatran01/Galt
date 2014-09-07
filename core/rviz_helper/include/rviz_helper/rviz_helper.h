#ifndef RVIZ_HELPER_H_
#define RVIZ_HELPER_H_

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace rviz_helper {

namespace colors {
const std::vector<double> RED = { 1, 0, 0 };
const std::vector<double> GREEN = { 0, 1, 0 };
const std::vector<double> BLUE = { 0, 0, 1 };
const std::vector<double> CYAN = { 0, 1, 1 };
const std::vector<double> MAGENTA = { 1, 0, 1 };
const std::vector<double> YELLOW = { 1, 1, 0 };
}

class TrajectoryVisualizer {
public:
  TrajectoryVisualizer(const ros::NodeHandle &nh,
                       const std::string &topic = std::string("traj"));

  void set_colorRGB(const std::vector<double> &rgb) {
    markers_.color.r = rgb[0];
    markers_.color.g = rgb[1];
    markers_.color.b = rgb[2];
    markers_.color.a = 1;
  }
  void set_scale(double scale) { markers_.scale.x = scale; }
  void set_lifetime(const ros::Duration &duration) {
    markers_.lifetime = duration;
  }

  void PublishTrajectory(const geometry_msgs::Point &point,
                         const std_msgs::Header &header);
  void PublishTrajectory(const geometry_msgs::Point &point,
                         const std::string &frame_id, const ros::Time &time);

private:
  ros::NodeHandle nh_;
  ros::Publisher traj_pub_;
  visualization_msgs::Marker markers_;
};

} // namespace rviz_helper

#endif // RVIZ_HELPER_H_
