#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>

#include "ndvi_preview/ndvi_cloud_node.hpp"

namespace ndvi_preview {

bool NdviCloudNode::SaveToPcd(SaveToPcd::Request &req,
                              SaveToPcd::Response &res) {
  laser_assembler::AssembleScans2 srv_red, srv_ir;
  // Call two services to get assembled red and ir point clouds
  if (AssembleCloud2(nh_, srv_red, assemble_red_srv_client_) &&
      AssembleCloud2(nh_, srv_ir, assemble_ir_srv_client_)) {
    res.success = true;
  } else {
    res.success = false;
  }
  return res.success;
}

bool AssembleCloud2(const ros::NodeHandle &nh,
                    laser_assembler::AssembleScans2 &srv,
                    ros::ServiceClient &client) {
  ROS_INFO_STREAM("Calling service: " << client.getService());
  srv.request.begin = ros::Time(0);
  srv.request.end = ros::Time::now();
  if (!client.call(srv)) {
    ROS_WARN_THROTTLE(1, "%s: Sercie call failed", nh.getNamespace().c_str());
    return false;
  }

  if (srv.response.cloud.data.empty()) {
    ROS_WARN_THROTTLE(1, "%s: Empty cloud", nh.getNamespace().c_str());
    return false;
  }

  ROS_INFO("Got cloud with %ld points", srv.response.cloud.data.size());
  return true;
}

}  // namespace ndvi

int main(int argc, char **argv) {
  ros::init(argc, argv, "ndvi_cloud");
  ros::NodeHandle nh("~");

  ndvi_preview::NdviCloudNode ndvi_cloud_node(nh);
  ros::spin();
}
