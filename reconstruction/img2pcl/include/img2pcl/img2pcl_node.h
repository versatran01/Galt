#ifndef GALT_IMG2PCL_NODE_H_
#define GALT_IMG2PCL_NODE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl_ros/point_cloud.h>
#include <boost/optional.hpp>

namespace galt {
namespace img2pcl {

class Img2pclNode {
 public:
  Img2pclNode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

  void CameraCb(const sensor_msgs::ImageConstPtr &image_msg,
                const sensor_msgs::CameraInfoConstPtr &cinfo_msg);
  void Cloud2Cb(const sensor_msgs::PointCloud2ConstPtr &cloud2_msg);
  void ConnectCb();

 private:
  boost::optional<geometry_msgs::TransformStamped> GetLatestTransfrom(
      const std::string &frame_tgt, const std::string &frame_src) const;

  int queue_size_;
  ros::NodeHandle nh_, pnh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
  ros::Subscriber sub_cloud2_;
  image_geometry::PinholeCameraModel camera_model_;
  boost::mutex connect_mutex_;
  ros::Publisher pub_cloud2_;
  tf2::BufferCore core_;
  tf2_ros::TransformListener tf_listener_;
  cv_bridge::CvImageConstPtr cv_ptr_;
};

pcl::PointCloud<pcl::PointXYZ> TransformCloud(
    const pcl::PointCloud<pcl::PointXYZ> &pcl_cloud_src,
    const geometry_msgs::TransformStamped &transform_stamped);

pcl::PointCloud<pcl::PointXYZRGB> ExtractColor(
    const pcl::PointCloud<pcl::PointXYZ> &pcl_cloud_src,
    const pcl::PointCloud<pcl::PointXYZ> &pcl_cloud_tgt, const cv::Mat &image,
    const image_geometry::PinholeCameraModel &model);

template <typename PclPointT>
std::vector<cv::Point3f> CloudToCvPoints3(
    const pcl::PointCloud<PclPointT> &pcl_cloud) {
  std::vector<cv::Point3f> cv_pts;
  for (const PclPointT &point : pcl_cloud.points) {
    cv_pts.emplace_back(point.x, point.y, point.z);
  }
  return cv_pts;
}

template <typename Scalar>
bool IsInsideImage(const cv::Point_<Scalar> &point, const cv::Size &size) {
  return (point.x >= 0) && (point.y >= 0) && (point.x < size.width) &&
         (point.y < size.height);
}

}  // namespace img2pcl
}  // namespace galt

#endif  // GALT_IMG2PCL_NODE_H_
