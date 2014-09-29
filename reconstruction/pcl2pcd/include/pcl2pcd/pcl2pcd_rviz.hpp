#ifndef PCL2PCD_RVIZ_HPP_
#define PCL2PCD_RVIZ_HPP_

#include <ros/ros.h>

namespace pcl2pcd {

class Pcl2PcdRviz {
 public:
  Pcl2PcdRviz(const ros::NodeHandle &nh);
private:
  ros::NodeHandle nh_;
};

}  // namespace pcl2pcd

#endif  // PCL2PCD_RVIZ_HPP_
