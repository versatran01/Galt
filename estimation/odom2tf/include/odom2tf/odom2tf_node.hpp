#ifndef ODOM2TF_ODOM2TF_NODE_HPP_
#define ODOM2TF_ODOM2TF_NODE_HPP_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>

namespace odom2tf {

class Odom2tfNode {

 public:
  Odom2tfNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  void OdomCallback(const nav_msgs::OdometryConstPtr& odom_msg);

 private:
  void PublishTransform(const geometry_msgs::Pose& pose,
                        const std_msgs::Header& header,
                        const std::string& child_frame_id);
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber odom_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

}  // namespace odom2tf

#endif  // ODOM2TF_ODOM2TF_NODE_HPP_
