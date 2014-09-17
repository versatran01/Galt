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
  constexpr static double kDelay = 0.05;

  void CameraCb(const sensor_msgs::ImageConstPtr &image_msg,
                const sensor_msgs::CameraInfoConstPtr &cinfo_msg);
  void ConnectCb();
  bool GetLatestTransfrom(const std::string &frame_tgt,
                          const std::string &frame_src,
                          geometry_msgs::TransformStamped *tf_stamped) const;
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

template <typename PclPointT>
void TransformCloud(const sensor_msgs::PointCloud2 &src_ros_cloud,
                    const geometry_msgs::Transform &transform,
                    const std::string &tgt_frame,
                    pcl::PointCloud<PclPointT> *tgt_pcl_cloud) {

  // Convert transform message to eigen
  Eigen::Affine3d affine;
  tf::transformMsgToEigen(transform, affine);
  // Convert point cloud msg
  pcl::PointCloud<PclPointT> cloud_pcl_src;
  pcl::fromROSMsg(src_ros_cloud, cloud_pcl_src);
  // Transform point cloud
  pcl::transformPointCloud(cloud_pcl_src, *tgt_pcl_cloud, affine);
  tgt_pcl_cloud->header.frame_id = tgt_frame;
}

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

template <typename PclPointT>
void ExtractInfoFromImage(const pcl::PointCloud<PclPointT> pcl_cloud_in,
                          const cv::Mat &image,
                          const image_geometry::PinholeCameraModel &model,
                          sensor_msgs::PointCloud2 *ros_cloud_out) {
  std::vector<cv::Point3f> cam_pts;
  std::vector<cv::Point2f> img_pts;
  CloudToPoints3(pcl_cloud_in, &cam_pts);
  cv::Mat zeros = cv::Mat::zeros(1, 3, CV_64FC1);
  cv::projectPoints(cam_pts, zeros, zeros, model.fullIntrinsicMatrix(),
                    model.distortionCoeffs(), img_pts);
  // Construct a new point cloud, super hacky, will need to refactor this shit
  pcl::PointCloud<pcl::PointXYZRGB> cloud_pcl_out;
  pcl::copyPointCloud(pcl_cloud_in, cloud_pcl_out);
  cloud_pcl_out.points.clear();
  //  cv::Mat color;
  //  cv::cvtColor(image, color, CV_GRAY2BGR);
  for (size_t i = 0; i < pcl_cloud_in.points.size(); ++i) {
    cv::Point2i pixel_rounded(img_pts[i].x, img_pts[i].y);
    if (IsInsideImage(pixel_rounded, image.cols, image.rows)) {
      pcl::PointXYZRGB point;
      point.x = pcl_cloud_in.points[i].x;
      point.y = pcl_cloud_in.points[i].y;
      point.z = pcl_cloud_in.points[i].z;
      const uchar *p = image.ptr<uchar>(pixel_rounded.y);
      int col = pixel_rounded.x;
      point.r = p[3 * col];
      point.g = p[3 * col + 1];
      point.b = p[3 * col + 2];
      cloud_pcl_out.points.push_back(point);
    }
  }
  cloud_pcl_out.width = cloud_pcl_out.points.size();
  pcl::toROSMsg(cloud_pcl_out, *ros_cloud_out);
  ros_cloud_out->header.frame_id = "mv_stereo/left";
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
