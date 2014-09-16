#ifndef GALT_IMG2PCL_NODE_H_
#define GALT_IMG2PCL_NODE_H_

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
namespace img2pcl {

class Img2pclNode {

 public:
  Img2pclNode(const ros::NodeHandle &nh);

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

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
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
