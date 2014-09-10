#ifndef RVIZ_HELPER_H_
#define RVIZ_HELPER_H_

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

namespace rviz_helper {

namespace colors {
const std::vector<double> RED = {1, 0, 0};
const std::vector<double> GREEN = {0, 1, 0};
const std::vector<double> BLUE = {0, 0, 1};
const std::vector<double> CYAN = {0, 1, 1};
const std::vector<double> MAGENTA = {1, 0, 1};
const std::vector<double> YELLOW = {1, 1, 0};
}

class TrajectoryVisualizer {
 public:
  TrajectoryVisualizer(const ros::NodeHandle &nh,
                       const std::string &topic = std::string("traj"));

  void set_colorRGB(const std::vector<double> &rgb) {
    markers_.color.r = rgb[0];
    markers_.color.g = rgb[1];
    markers_.color.b = rgb[2];
    if (rgb.size() == 4) {
      markers_.color.a = rgb[3];
    } else {
      markers_.color.a = 1;
    }
  }
  void set_scale(double scale) { markers_.scale.x = scale; }
  void set_lifetime(const ros::Duration &duration) {
    markers_.lifetime = duration;
  }
  void set_num_skip(int num_skip) { num_skip_ = num_skip; }

  void PublishTrajectory(const geometry_msgs::Point &point,
                         const std_msgs::Header &header);
  void PublishTrajectory(const geometry_msgs::Point &point,
                         const std::string &frame_id, const ros::Time &time);

 private:
  ros::NodeHandle nh_;
  ros::Publisher traj_pub_;
  int num_skip_;
  int total_points_cnt_;
  visualization_msgs::Marker markers_;
};

/**
 * @brief Visualize covariance as a scaled ellipsoid.
 */
class CovarianceVisualizer {
 public:
  CovarianceVisualizer(const ros::NodeHandle &nh,
                       const std::string &topic = std::string("covariance"));

  void set_colorRGB(const std::vector<double> &rgb) {
    marker_.color.r = rgb[0];
    marker_.color.g = rgb[1];
    marker_.color.b = rgb[2];
    if (rgb.size() == 4) {
      marker_.color.a = rgb[3];
    } else {
      marker_.color.a = 1;
    }
  }

  void PublishCovariance(const geometry_msgs::PoseWithCovariance &pose_cov,
                         const std_msgs::Header &header);
  void PublishCovariance(
      const geometry_msgs::PoseWithCovarianceStamped &pose_cov_stamped);
  void PublishCovariance(const nav_msgs::Odometry &odometry);

 private:
  ros::NodeHandle nh_;
  ros::Publisher cov_pub_;
  visualization_msgs::Marker marker_;
};

}  // namespace rviz_helper

#endif  // RVIZ_HELPER_H_
