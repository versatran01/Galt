#ifndef PCL2PCD_NODE_HPP_
#define PCL2PCD_NODE_HPP_

#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include <pcl2pcd/SaveToPcd.h>
#include <pcl2pcd/ResetTime.h>

namespace pcl2pcd {

class Pcl2PcdNode {
 public:
  Pcl2PcdNode(const ros::NodeHandle& nh)
      : nh_(nh),
        prev_stamp_(0.0f),
        save_srv_server_(
            nh_.advertiseService("save_to_pcd", &Pcl2PcdNode::SaveToPcd, this)),
        reset_srv_server_(
            nh_.advertiseService("reset_time", &Pcl2PcdNode::ResetTime, this)),
        srv_client_(nh_.serviceClient<laser_assembler::AssembleScans2>(
            "assemble_scans2")) {}

 private:
  bool SaveToPcd(pcl2pcd::SaveToPcd::Request& req,
                 pcl2pcd::SaveToPcd::Response& res);
  bool ResetTime(pcl2pcd::ResetTime::Request& req,
                 pcl2pcd::ResetTime::Response& res);

  ros::NodeHandle nh_;  ///< Node handle of assembler
  ros::Time prev_stamp_;
  ros::ServiceServer save_srv_server_;
  ros::ServiceServer reset_srv_server_;
  ros::ServiceClient srv_client_;
};

}  // namespace pcl2pcd

#endif  // PCL2PCD_NODE_HPP_
