#ifndef PCL2PCD_RVIZ_HPP_
#define PCL2PCD_RVIZ_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace pcl2pcd {

class Pcl2PcdRviz {
 public:
  Pcl2PcdRviz(const ros::NodeHandle& nh);

 private:
  void GoalCb(const geometry_msgs::PoseStampedConstPtr& pose_stamped);

  ros::NodeHandle nh_;
  ros::Subscriber sub_goal_;
  ros::Subscriber sub_scan_;
};

}  // namespace pcl2pcd

#endif  // PCL2PCD_RVIZ_HPP_
