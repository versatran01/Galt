#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace galt {
namespace iris_tf {
/**
 * @brief The IrisTransform class
 */
class IrisTransform {
 public:
  IrisTransform(const ros::NodeHandle &nh);

 private:
  void OdomCb(const nav_msgs::OdometryConstPtr &odom_msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_odom_;
  std::string frame_;
  tf2_ros::TransformBroadcaster broadcaster_;
};

IrisTransform::IrisTransform(const ros::NodeHandle &nh) : nh_{nh} {
  nh_.param<std::string>("frame", frame_, "imu");
  sub_odom_ = nh_.subscribe("topic", 1, &IrisTransform::OdomCb, this);
}

void IrisTransform::OdomCb(const nav_msgs::OdometryConstPtr &odom_msg) {
  const geometry_msgs::Point &position = odom_msg->pose.pose.position;
  const geometry_msgs::Quaternion &quaternion = odom_msg->pose.pose.orientation;

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
