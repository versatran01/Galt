#ifndef GALT_NDVI_PREVIEW_NDVI_CLOUD_HPP_
#define GALT_NDVI_PREVIEW_NDVI_CLOUD_HPP_

#include <ros/ros.h>

#include <laser_assembler/AssembleScans2.h>
#include <ndvi_preview/SaveToPcd.h>

namespace ndvi_preview {

class NdviCloudNode {
 public:
  NdviCloudNode(const ros::NodeHandle &nh)
      : nh_(nh),
        save_srv_server_(nh_.advertiseService("save_to_pcd",
                                              &NdviCloudNode::SaveToPcd, this)),
        assemble_red_srv_client_(
            nh_.serviceClient<laser_assembler::AssembleScans2>("assemble_red")),
        assemble_ir_srv_client_(
            nh_.serviceClient<laser_assembler::AssembleScans2>("assemble_ir")) {
  }

  bool SaveToPcd(SaveToPcd::Request &req, SaveToPcd::Response &res);

 private:
  ros::NodeHandle nh_;
  ros::ServiceServer save_srv_server_;
  ros::ServiceClient assemble_red_srv_client_;
  ros::ServiceClient assemble_ir_srv_client_;
};

bool AssembleCloud2(const ros::NodeHandle &nh,
                    laser_assembler::AssembleScans2 &srv,
                    ros::ServiceClient &client);

}  // namespace ndvi

#endif  // GALT_NDVI_PREVIEW_NDVI_CLOUD_HPP_
