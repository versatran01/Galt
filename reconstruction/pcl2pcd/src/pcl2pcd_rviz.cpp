#include "pcl2pcd/pcl2pcd_rviz.hpp"

namespace pcl2pcd {

Pcl2PcdRviz::Pcl2PcdRviz(const ros::NodeHandle &nh)
    : nh_(nh),
      sub_goal_(nh_.subscribe("/move_base_simple/goal", 1, &Pcl2PcdRviz::GoalCb,
                              this)) {}

void Pcl2PcdRviz::GoalCb(
    const geometry_msgs::PoseStampedConstPtr &pose_stamped) {
  ROS_INFO("Goal recieved");
}
}  // namespace pcl2pcd
