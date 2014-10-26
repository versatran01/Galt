#ifndef GALT_NDVI_PREVIEW_NDVI_CLOUD_HPP_
#define GALT_NDVI_PREVIEW_NDVI_CLOUD_HPP_

#include <ros/ros.h>

#include <laser_assembler/AssembleScans2.h>
#include <std_srvs/Empty.h>
#include <ndvi_preview/SaveToPcd.h>
#include <ndvi_preview/LoadFromPcd.h>

namespace ndvi_preview {

class NdviCloudNode {
 public:
  NdviCloudNode(const ros::NodeHandle &nh)
      : nh_(nh),
        save_srv_server_(nh_.advertiseService("save_to_pcd",
                                              &NdviCloudNode::SaveToPcd, this)),
        ndvi_srv_server_(nh_.advertiseService(
            "generate_ndvi", &NdviCloudNode::GenerateNdvi, this)),
        assemble_red_srv_client_(
            nh_.serviceClient<laser_assembler::AssembleScans2>("assemble_red")),
        assemble_ir_srv_client_(
            nh_.serviceClient<laser_assembler::AssembleScans2>("assemble_ir")),
        ndvi_pub_(nh_.advertise<sensor_msgs::PointCloud2>("ndvi", 1, true)) {}

  bool GenerateNdvi(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &res);
  bool SaveToPcd(SaveToPcd::Request &req, SaveToPcd::Response &res);
  bool AssembleCloud2(laser_assembler::AssembleScans2 &srv,
                      ros::ServiceClient &client);
  bool SavePcdToDir(const sensor_msgs::PointCloud2 &cloud_ros,
                    const std::string &dirname, const std::string &filename);

 private:
  ros::NodeHandle nh_;
  ros::ServiceServer save_srv_server_;
  ros::ServiceServer ndvi_srv_server_;
  ros::ServiceClient assemble_red_srv_client_;
  ros::ServiceClient assemble_ir_srv_client_;
  ros::Publisher ndvi_pub_;
};

}  // namespace ndvi

#endif  // GALT_NDVI_PREVIEW_NDVI_CLOUD_HPP_
