#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "ndvi_preview/ndvi_cloud_node.hpp"

namespace ndvi_preview {

bool NdviCloudNode::SaveToPcd(SaveToPcd::Request &req,
                              SaveToPcd::Response &res) {
  laser_assembler::AssembleScans2 srv_red, srv_ir;
  // Call two services to get assembled red and ir point clouds
  if (AssembleCloud2(srv_red, assemble_red_srv_client_) &&
      AssembleCloud2(srv_ir, assemble_ir_srv_client_)) {
    if (SavePcdToDir(srv_red.response.cloud, req.dirname, "red.pcd") &&
        SavePcdToDir(srv_ir.response.cloud, req.dirname, "ir.pcd")) {
      res.success = true;
      return res.success;
    }
    res.success = false;
    ROS_INFO("Something bad happend");
    return res.success;
  }
}

bool NdviCloudNode::AssembleCloud2(laser_assembler::AssembleScans2 &srv,
                                   ros::ServiceClient &client) {
  ROS_INFO_STREAM("Calling service: " << client.getService());
  srv.request.begin = ros::Time(0);
  srv.request.end = ros::Time::now();
  if (!client.call(srv)) {
    ROS_WARN_THROTTLE(1, "%s: Sercie call failed", nh_.getNamespace().c_str());
    return false;
  }

  if (srv.response.cloud.data.empty()) {
    ROS_WARN_THROTTLE(1, "%s: Empty cloud", nh_.getNamespace().c_str());
    return false;
  }

  ROS_INFO("Got cloud with %ld points", srv.response.cloud.data.size());
  return true;
}

bool NdviCloudNode::SavePcdToDir(const sensor_msgs::PointCloud2 &cloud_ros,
                                 const std::string &dirname,
                                 const std::string &filename) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud_pcl_w;
  pcl::fromROSMsg(cloud_ros, cloud_pcl_w);
  std::string full_filename = dirname + "/" + filename;
  try {
    ROS_INFO_STREAM("Saving cloud to " << full_filename);
    pcl::io::savePCDFile(full_filename, cloud_pcl_w);
    ROS_INFO_STREAM("Cloud saved to" << full_filename);
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", nh_.getNamespace().c_str(), e.what());
    return false;
  }
  return true;
}

}  // namespace ndvi

int main(int argc, char **argv) {
  ros::init(argc, argv, "ndvi_cloud");
  ros::NodeHandle nh("~");

  ndvi_preview::NdviCloudNode ndvi_cloud_node(nh);
  ros::spin();
}
