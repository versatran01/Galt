/*
 * thermal_map_node.h
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of thermal_map.
 *
 *	Created on: 23/08/2014
 */

#ifndef GALT_THERMAL_MAP_NODE_H_
#define GALT_THERMAL_MAP_NODE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>

namespace galt {
namespace thermal_map {

class ThermalMapNode {

 public:
  ThermalMapNode(const ros::NodeHandle &nh);

 private:
  constexpr static double kDelay = 0.03;

  void CameraCb(const sensor_msgs::ImageConstPtr &image_msg,
                const sensor_msgs::CameraInfoConstPtr &cinfo_msg);
  void ConnectCb();
  void ProjectCloud(const sensor_msgs::PointCloud &cloud_in,
                    const cv::Mat &image,
                    const image_geometry::PinholeCameraModel &model,
                    sensor_msgs::PointCloud &cloud_out) const;
  void PixelsToCloud(const cv::Mat &image,
                     const std::vector<cv::Point2f> &pixels,
                     const sensor_msgs::PointCloud cloud_in,
                     sensor_msgs::PointCloud &cloud_out) const;
  void CloudToPoints(const sensor_msgs::PointCloud &cloud,
                     std::vector<cv::Point3f> &points) const;

  void ImageCb(const sensor_msgs::ImageConstPtr &image_msg);

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
  image_transport::Subscriber sub_image_;
  ros::Publisher pub_cloud_;
  image_geometry::PinholeCameraModel camera_model_;
  ros::ServiceClient client_;
};

template <typename T>
bool InsideImage(const cv::Size &size, const cv::Point_<T> &pixel) {
  return (pixel.x >= 0) && (pixel.y >= 0) && (pixel.x <= size.width) &&
         (pixel.y <= size.height);
}

}  // namespace thermal_map
}  // namespace galt
#endif  // GALT_THERMAL_MAP_NODE_H_
