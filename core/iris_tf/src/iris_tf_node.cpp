#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace galt {
namespace iris_tf {

class IrisTransform {
 public:
  IrisTransform(const ros::NodeHandle &nh);

 private:
  void OdomCb(const nav_msgs::OdometryConstPtr &odom_msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_odom_;
  ros::Publisher pub_path_;
  std::string frame_;
  tf2_ros::TransformBroadcaster broadcaster_;
  nav_msgs::Path path_;
};

IrisTransform::IrisTransform(const ros::NodeHandle &nh)
    : nh_{nh},
      sub_odom_(nh_.subscribe("topic", 1, &IrisTransform::OdomCb, this)),
      pub_path_(nh_.advertise<nav_msgs::Path>("path", 1)) {
  nh_.param<std::string>("frame", frame_, "imu");
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

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.pose = odom_msg->pose.pose;
  pose_stamped.header = odom_msg->header;
  path_.header = odom_msg->header;
  path_.poses.push_back(pose_stamped);
  pub_path_.publish(path_);

  broadcaster_.sendTransform(transform_stamped);
}

}  // namespace iris_tf
}  // namespace galt

int main(int argc, char **argv) {
  ros::init(argc, argv, "iris_tf");
  ros::NodeHandle nh("~");

  galt::iris_tf::IrisTransform iris_tf(nh);
  ros::spin();
  return 0;
}
