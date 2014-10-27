#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "ndvi_preview/ndvi_cloud_node.hpp"

namespace ndvi_preview {

bool NdviCloudNode::GenerateNdvi(std_srvs::Empty::Request &req,
                                 std_srvs::Empty::Response &res) {
  laser_assembler::AssembleScans2 srv_red, srv_ir;
  // Call two services to get assembled red and ir point clouds
  if (AssembleCloud2(srv_red, assemble_red_srv_client_) &&
      AssembleCloud2(srv_ir, assemble_ir_srv_client_)) {
    const int K = 1;

    // Convert to pcl point cloud type
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_red(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ir(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ndvi(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    // cloud ndvi should have the same size as cloud red
    cloud_ndvi->height = 1;
    cloud_ndvi->points.reserve(cloud_red->width);
    pcl_conversions::toPCL(srv_red.response.cloud.header, cloud_ndvi->header);

    pcl::fromROSMsg(srv_red.response.cloud, *cloud_red);
    pcl::fromROSMsg(srv_ir.response.cloud, *cloud_ir);

    // Iterate through all points in red and find the 1nn in ir
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud_ir);
    for (size_t i = 0; i < cloud_red->width; ++i) {
      const pcl::PointXYZRGB &red_point = cloud_red->points[i];
      std::vector<int> point_1nn(K);
      std::vector<float> point_1nn_dist2(K);
      if (kdtree.nearestKSearch(red_point, K, point_1nn, point_1nn_dist2)) {
        const pcl::PointXYZRGB &ir_point = cloud_ir->points[i];
        pcl::PointXYZRGB ndvi_point(red_point);
        auto ndvi = static_cast<float>(ir_point.r - red_point.r) /
                    (ir_point.r + red_point.r);
        ndvi = (ndvi + 1) / 2 * 255;
        auto ndvi_rgb = static_cast<uint8_t>(ndvi);
        ndvi_point.r = ndvi_rgb;
        ndvi_point.g = ndvi_rgb;
        ndvi_point.b = ndvi_rgb;
        cloud_ndvi->points.push_back(ndvi_point);
        cloud_ndvi->width += 1;
      }
    }

    ROS_INFO_STREAM("number of ndvi: " << cloud_ndvi->width
                                       << " number of red: "
                                       << cloud_red->width);
    // Convert ndvi cloud to PointCloud2 and publish
    sensor_msgs::PointCloud2 cloud_ndvi_msg;
    pcl::toROSMsg(*cloud_ndvi, cloud_ndvi_msg);
    ndvi_pub_.publish(cloud_ndvi_msg);
    return true;
  }
  return false;
}

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
    ROS_INFO("Failed to save to pcd");
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
