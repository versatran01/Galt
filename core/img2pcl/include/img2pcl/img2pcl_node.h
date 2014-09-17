#ifndef GALT_IMG2PCL_NODE_H_
#define GALT_IMG2PCL_NODE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Geometry>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

namespace galt {
namespace img2pcl {

class Img2pclNode {

 public:
  Img2pclNode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

 private:
  constexpr static double kDelay = 0.03;

  void CameraCb(const sensor_msgs::ImageConstPtr &image_msg,
                const sensor_msgs::CameraInfoConstPtr &cinfo_msg);
  void ConnectCb();
  bool GetLatestTransfrom(const std::string &target_frame,
                          const std::string &source_frame,
                          geometry_msgs::TransformStamped *tf_stamped);
  //  void ProjectCloud(const sensor_msgs::PointCloud &cloud_in,
  //                    const cv::Mat &image,
  //                    const image_geometry::PinholeCameraModel &model,
  //                    sensor_msgs::PointCloud &cloud_out) const;
  //  void PixelsToCloud(const cv::Mat &image,
  //                     const std::vector<cv::Point2f> &pixels,
  //                     const sensor_msgs::PointCloud cloud_in,
  //                     sensor_msgs::PointCloud &cloud_out) const;
  //  void CloudToPoints(const sensor_msgs::PointCloud &cloud,
  //                     std::vector<cv::Point3f> &points) const;

  ros::NodeHandle nh_, pnh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
  image_geometry::PinholeCameraModel camera_model_;
  ros::Publisher pub_cloud_;
  ros::ServiceClient srv_client_;
  tf2::BufferCore core_;
  tf2_ros::TransformListener tf_listener_;
};

template <typename PointType>
void TransformCloud(const sensor_msgs::PointCloud2 &cloud_msg_in,
                    const geometry_msgs::Transform &transform,
                    pcl::PointCloud<PointType> &cloud_pcl_out) {

  // Convert transform message to eigen
  Eigen::Affine3d affine;
  tf::transformMsgToEigen(transform, affine);
  // Convert point cloud msg
  pcl::PointCloud<PointType> cloud_pcl_in;
  pcl::fromROSMsg(cloud_msg_in, cloud_pcl_in);
  // Transform point cloud
  pcl::transformPointCloud(cloud_pcl_in, cloud_pcl_out, affine);
}

/*
template <typename T>
bool InsideImage(const cv::Size &size, const cv::Point_<T> &pixel) {
  return (pixel.x >= 0) && (pixel.y >= 0) && (pixel.x <= size.width) &&
         (pixel.y <= size.height);
}
*/

}  // namespace thermal_map
}  // namespace galt

#endif  // GALT_THERMAL_MAP_NODE_H_
