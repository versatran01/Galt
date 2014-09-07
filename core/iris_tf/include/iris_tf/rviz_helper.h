#ifndef GALT_IRIS_TF_RVIZ_HELPER_H_
#define GALT_IRIS_TF_RVIZ_HELPER_H_

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace galt {
namespace iris_tf {

class TrajectoryVisualizer {
 public:
  TrajectoryVisualizer(const ros::NodeHandle &nh);

  void set_colorRGBA(const std::vector<double> &rgba) {
    markers_.color.r = rgba[0];
    markers_.color.g = rgba[1];
    markers_.color.b = rgba[2];
    markers_.color.a = rgba[3];
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

}  // namespace iris_tf
}  // namespace galt

#endif  // GALT_IRIS_TF_RVIZ_HELPER_H_
