#include "img2pcl/img2pcl_node.h"

#include <laser_assembler/AssembleScans2.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace galt {
namespace img2pcl {

using namespace pcl;
using namespace geometry_msgs;

Img2pclNode::Img2pclNode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), it_(nh), tf_listener_(core_) {
  pnh_.param("queue_size", queue_size_, 1);
  ros::SubscriberStatusCallback connect_cb =
      boost::bind(&Img2pclNode::ConnectCb, this);
  pub_cloud2_ = pnh_.advertise<sensor_msgs::PointCloud2>(
      "color_cloud2", queue_size_, connect_cb, connect_cb);
}

void Img2pclNode::ConnectCb() {
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_cloud2_.getNumSubscribers() == 0) {
    sub_camera_.shutdown();
    sub_cloud2_.shutdown();
  } else if (!sub_camera_ || !sub_cloud2_) {
    image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
    const std::string image_topic = pnh_.resolveName("image");
    sub_camera_ = it_.subscribeCamera(image_topic, queue_size_,
                                      &Img2pclNode::CameraCb, this, hints);
    ROS_INFO("Subscribing to %s", image_topic.c_str());
    const std::string cloud2_topic = pnh_.resolveName("cloud2");
    sub_cloud2_ =
        pnh_.subscribe(cloud2_topic, queue_size_, &Img2pclNode::Cloud2Cb, this);
    ROS_INFO("Subscribing to %s", cloud2_topic.c_str());
  }
}

void Img2pclNode::Cloud2Cb(const sensor_msgs::PointCloud2ConstPtr &cloud2_msg) {
  if (!cv_ptr_) {
    ROS_INFO("Waiting for the first image");
    return;
  }
  // Get transform from laser cloud to image, this should be a fixed transform
  if (const auto tf_stamped_o = GetLatestTransfrom(
          cv_ptr_->header.frame_id, cloud2_msg->header.frame_id)) {
    const TransformStamped tf_stamped = *tf_stamped_o;
    // Convert cloud2_msg into a pcl point cloud
    PointCloud<PointXYZ> pcl_cloud_src;
    pcl::fromROSMsg(*cloud2_msg, pcl_cloud_src);
    // Transform src pcl cloud into camera frame
    PointCloud<PointXYZ> pcl_cloud_tgt =
        TransformCloud(pcl_cloud_src, tf_stamped);
    // Project pcl point into pixel value and filter based on image size
    PointCloud<PointXYZRGB> pcl_cloud_src_color = ExtractColor(
        pcl_cloud_src, pcl_cloud_tgt, cv_ptr_->image, camera_model_);
    pub_cloud2_.publish(pcl_cloud_src_color);
  }
}

void Img2pclNode::CameraCb(const sensor_msgs::ImageConstPtr &image_msg,
                           const sensor_msgs::CameraInfoConstPtr &cinfo_msg) {
  if (cinfo_msg->K[0] == 0.0 || cinfo_msg->height == 0 ||
      cinfo_msg->width == 0) {
    ROS_WARN_THROTTLE(1, "Uncalibrated camera.");
    return;
  }
  camera_model_.fromCameraInfo(cinfo_msg);
  // Save the latest image to be used in Cloud2Cb
  cv_ptr_ = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
}

boost::optional<TransformStamped> Img2pclNode::GetLatestTransfrom(
    const std::string &frame_tgt, const std::string &frame_src) const {
  try {
    return core_.lookupTransform(frame_tgt, frame_src, ros::Time(0));
  } catch (const tf2::TransformException &e) {
    ROS_WARN("%s", e.what());
    return boost::none;
  }
}

PointCloud<PointXYZ> TransformCloud(const PointCloud<PointXYZ> &pcl_cloud_src,
                                    const TransformStamped &transform_stamped) {
  // Convert transform message to eigen
  Eigen::Affine3d affine;
  tf::transformMsgToEigen(transform_stamped.transform, affine);
  // Transform point cloud into target frame
  PointCloud<PointXYZ> pcl_cloud_tgt;
  pcl::transformPointCloud(pcl_cloud_src, pcl_cloud_tgt, affine);
  pcl_cloud_tgt.header.frame_id = transform_stamped.header.frame_id;
  return pcl_cloud_tgt;
}

PointCloud<PointXYZRGB> ExtractColor(
    const PointCloud<PointXYZ> &pcl_cloud_src,
    const PointCloud<PointXYZ> &pcl_cloud_tgt, const cv::Mat &image,
    const image_geometry::PinholeCameraModel &model) {
  ROS_ASSERT_MSG(pcl_cloud_src.size() == pcl_cloud_tgt.size(),
                 "point size mismatch");

  std::vector<cv::Point3f> cam_pts = CloudToCvPoints3(pcl_cloud_tgt);
  std::vector<cv::Point2f> img_pts;
  cv::Mat zeros = cv::Mat::zeros(1, 3, CV_64FC1);
  cv::projectPoints(cam_pts, zeros, zeros, model.fullIntrinsicMatrix(),
                    model.distortionCoeffs(), img_pts);

  PointCloud<PointXYZRGB> pcl_cloud_src_color;
  copyPointCloud(pcl_cloud_src, pcl_cloud_src_color);
  pcl_cloud_src_color.points.clear();
  for (size_t i = 0; i < pcl_cloud_src.size(); ++i) {
    cv::Point2i pixel_rounded(img_pts[i].x, img_pts[i].y);
    if (IsInsideImage(pixel_rounded, image.size())) {
      PointXYZRGB point;
      point.x = pcl_cloud_src.points[i].x;
      point.y = pcl_cloud_src.points[i].y;
      point.z = pcl_cloud_src.points[i].z;
      const uchar *p = image.ptr<uchar>(pixel_rounded.y);
      const int col = pixel_rounded.x;
      point.r = p[3 * col + 2];
      point.g = p[3 * col + 1];
      point.b = p[3 * col + 0];
      pcl_cloud_src_color.points.push_back(point);
    }
  }
  // Update width with new point size
  pcl_cloud_src_color.width = pcl_cloud_src_color.size();
  return pcl_cloud_src_color;
}

}  // namespace img2pcl
}  // namespace galt
