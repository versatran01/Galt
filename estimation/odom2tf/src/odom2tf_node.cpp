#include "odom2tf/odom2tf_node.hpp"

namespace odom2tf {

Odom2tfNode::Odom2tfNode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
  odom_sub_ = pnh_.subscribe("odom", 10, &Odom2tfNode::OdomCallback, this);
}

void Odom2tfNode::OdomCallback(const nav_msgs::OdometryConstPtr &odom_msg) {
  PublishTransform(odom_msg->pose.pose, odom_msg->header,
                   odom_msg->child_frame_id);
}

void Odom2tfNode::PublishTransform(const geometry_msgs::Pose &pose,
                                   const std_msgs::Header &header,
                                   const std::string &child_frame_id) {
  // Publish tf
  geometry_msgs::Vector3 translation;
  translation.x = pose.position.x;
  translation.y = pose.position.y;
  translation.z = pose.position.z;

  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header = header;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform.translation = translation;
  transform_stamped.transform.rotation = pose.orientation;

  tf_broadcaster_.sendTransform(transform_stamped);
}

}  // namespace odom2tf

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom2tf_node");
  ros::NodeHandle nh, pnh("~");

  odom2tf::Odom2tfNode odom2tf_node(nh, pnh);
  ros::spin();
}
