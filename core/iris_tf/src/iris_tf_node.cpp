#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "iris_tf/rviz_helper.h"

namespace galt {
namespace iris_tf {
/**
 * @brief The IrisTransform class
 */
class IrisTransform {
 public:
  /**
   * @brief IrisTransform Constructor
   * @param nh Private node handle
   */
  IrisTransform(const ros::NodeHandle &nh);

 private:
  /**
   * @brief OdomCb Odometry callback
   * @param odom_msg nav_msgs::Odometry
   */
  void OdomCb(const nav_msgs::OdometryConstPtr &odom_msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_odom_;
  ros::Publisher pub_traj_;
  std::string frame_;
  tf2_ros::TransformBroadcaster broadcaster_;
  TrajectoryVisualizer viz_traj_;
};

IrisTransform::IrisTransform(const ros::NodeHandle &nh) : nh_{nh} {
  nh_.param<std::string>("frame", frame_, "imu");
  sub_odom_ = nh_.subscribe("topic", 1, &IrisTransform::OdomCb, this);

  pub_traj_ = nh_.advertise<visualization_msgs::Marker>("trajectory", 1);
  std_msgs::ColorRGBA traj_color;
  traj_color.r = 1;
  traj_color.g = 1;
  traj_color.a = 1;
  viz_traj_ = TrajectoryVisualizer(pub_traj_, traj_color, 0.05, "line");
}

void IrisTransform::OdomCb(const nav_msgs::OdometryConstPtr &odom_msg) {
  const geometry_msgs::Point &position = odom_msg->pose.pose.position;
  const geometry_msgs::Quaternion &quaternion = odom_msg->pose.pose.orientation;

  // Don't really know what's the difference between a point and a vector3
  geometry_msgs::Vector3 translation;
  translation.x = position.x;
  translation.y = position.y;
  translation.z = position.z;

  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header = odom_msg->header;
  transform_stamped.child_frame_id = frame_;
  transform_stamped.transform.translation = translation;
  transform_stamped.transform.rotation = quaternion;

  broadcaster_.sendTransform(transform_stamped);
  viz_traj_.PublishTrajectory(odom_msg->pose.pose.position, odom_msg->header);
}

}  // namespace iris_tf
}  // namespace galt

/**
 * @brief main
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "iris_tf");
  ros::NodeHandle nh("~");

  galt::iris_tf::IrisTransform iris_tf(nh);
  ros::spin();
  return 0;
}
