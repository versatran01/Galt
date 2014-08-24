#ifndef GALT_THERMAL_MAP_VIZ_HELPER_H_
#define GALT_THERMAL_MAP_VIZ_HELPER_H_

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace galt {
namespace thermal_map {

class TrajectoryVisualizer {
 public:
  TrajectoryVisualizer() = default;
  TrajectoryVisualizer(const ros::Publisher &pub,
                       const std_msgs::ColorRGBA &color, const double scale,
                       const std::string &type);

  void PublishTrajectory(const geometry_msgs::Point &point,
                         const std_msgs::Header &header);
  void PublishTrajectory(const geometry_msgs::Point &point,
                         const std::string &frame_id, const ros::Time &time);

 private:
  ros::Publisher pub_;
  visualization_msgs::Marker markers_;
};

}  // namespace thermal_map
}  // namespace galt

#endif  // GALT_THERMAL_MAP_VIZ_HELPER_H_
