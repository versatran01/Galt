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

#include <opencv2/highgui/highgui.hpp>

namespace galt {
namespace img2pcl {

class Img2pclNode {

 public:
  Img2pclNode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

 private:
  constexpr static double kDelay = 0.1;

  void CameraCb(const sensor_msgs::ImageConstPtr &image_msg,
                const sensor_msgs::CameraInfoConstPtr &cinfo_msg);
  void ConnectCb();
  bool GetLatestTransfrom(const std::string &frame_tgt,
                          const std::string &frame_src,
                          geometry_msgs::TransformStamped *tf_stamped) const;

  ros::NodeHandle nh_, pnh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
  image_geometry::PinholeCameraModel camera_model_;
  ros::Publisher pub_cloud_;
  ros::ServiceClient srv_client_;
  tf2::BufferCore core_;
  tf2_ros::TransformListener tf_listener_;
};

template <typename PclPointT, typename Scalar>
void CloudToPoints3(const pcl::PointCloud<PclPointT> &pcl_cloud,
                    std::vector<cv::Point3_<Scalar>> *cv_pts) {
  for (const PclPointT &point : pcl_cloud.points) {
    cv_pts->emplace_back(point.x, point.y, point.z);
  }
}

template <typename Scalar>
bool IsInsideImage(cv::Point_<Scalar> point, int width, int height) {
  return (point.x >= 0) && (point.y >= 0) && (point.x < width) &&
         (point.y < height);
}

void TransformCloud(const sensor_msgs::PointCloud2 &src_ros_cloud,
                    const geometry_msgs::Transform &transform,
                    const std::string &tgt_frame,
                    pcl::PointCloud<pcl::PointXYZ> *tgt_pcl_cloud);

void ExtractInfoFromImage(const pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in,
                          const cv::Mat &image,
                          const image_geometry::PinholeCameraModel &model,
                          sensor_msgs::PointCloud2 *ros_cloud_out);

}  // namespace thermal_map
}  // namespace galt

#endif  // GALT_THERMAL_MAP_NODE_H_
