#include "img2pcl/img2pcl_node.h"

#include <laser_assembler/AssembleScans2.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace galt {
namespace img2pcl {

using namespace pcl;

Img2pclNode::Img2pclNode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), it_(nh), tf_listener_(core_) {
  std::string image;
  pnh_.param<std::string>("image", image, "");
  image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);
  sub_camera_ =
      it_.subscribeCamera(image, 1, &Img2pclNode::CameraCb, this, hints);
  srv_client_ = nh_.serviceClient<laser_assembler::AssembleScans2>("assemble");
  pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 1);
}

// Implement this later, or never
void Img2pclNode::ConnectCb() {}

void Img2pclNode::CameraCb(const sensor_msgs::ImageConstPtr &image_msg,
                           const sensor_msgs::CameraInfoConstPtr &cinfo_msg) {
  if (cinfo_msg->K[0] == 0.0 || cinfo_msg->height == 0) {
    ROS_WARN_THROTTLE(1, "Uncalibrated camera.");
    return;
  }
  camera_model_.fromCameraInfo(cinfo_msg);

  // Get point cloud from laser assembler
  static ros::Time prev_time(0);
  if (prev_time == ros::Time(0)) {
    prev_time = image_msg->header.stamp - ros::Duration(kDelay);
    return;
  }
  laser_assembler::AssembleScans2 srv;
  const ros::Time &curr_time = image_msg->header.stamp - ros::Duration(kDelay);
  srv.request.end = curr_time;
  srv.request.begin = prev_time;
  prev_time = curr_time;

  if (!srv_client_.call(srv)) {
    ROS_WARN_THROTTLE(1, "Sercie call failed");
    return;
  }

  if (srv.response.cloud.data.empty()) {
    ROS_WARN("Empty cloud");
    return;
  }

  // Transform world cloud msg to camera frame pcl point cloud
  geometry_msgs::TransformStamped tf_stamped;
  if (!GetLatestTransfrom(image_msg->header.frame_id, "world", &tf_stamped)) {
    return;
  }

  // Convert ros PointCloud2 to pcl PointCloud<T>
  const sensor_msgs::PointCloud2 &cloud_ros_w = srv.response.cloud;
  PointCloud<PointXYZ> cloud_pcl_c;
  TransformCloud(cloud_ros_w, tf_stamped.transform, image_msg->header.frame_id,
                 &cloud_pcl_c);

  // Back project pcl PointCloud<T> on to image and modify rgb/intensity value
  const cv::Mat image =
      cv_bridge::toCvShare(image_msg, image_msg->encoding)->image;
  sensor_msgs::PointCloud2 cloud_ros_c;
  ExtractInfoFromImage(cloud_pcl_c, image, camera_model_, &cloud_ros_c);
  pub_cloud_.publish(cloud_ros_c);
}

bool Img2pclNode::GetLatestTransfrom(
    const std::string &frame_tgt, const std::string &frame_src,
    geometry_msgs::TransformStamped *tf_stamped) const {
  try {
    *tf_stamped = core_.lookupTransform(frame_tgt, frame_src, ros::Time(0));
    return true;
  }
  catch (const tf2::TransformException &e) {
    ROS_WARN_THROTTLE(1, "%s", e.what());
    return false;
  }
}

void TransformCloud(const sensor_msgs::PointCloud2 &src_ros_cloud,
                    const geometry_msgs::Transform &transform,
                    const std::string &tgt_frame,
                    PointCloud<PointXYZ> *tgt_pcl_cloud) {
  // Convert transform message to eigen
  Eigen::Affine3d affine;
  tf::transformMsgToEigen(transform, affine);
  // Convert point cloud msg
  PointCloud<PointXYZ> cloud_pcl_src;
  pcl::fromROSMsg(src_ros_cloud, cloud_pcl_src);
  // Transform point cloud
  pcl::transformPointCloud(cloud_pcl_src, *tgt_pcl_cloud, affine);
  tgt_pcl_cloud->header.frame_id = tgt_frame;
}

void ExtractInfoFromImage(const PointCloud<PointXYZ> pcl_cloud_in,
                          const cv::Mat &image,
                          const image_geometry::PinholeCameraModel &model,
                          sensor_msgs::PointCloud2 *ros_cloud_out) {
  // Back project point in camera frame onto image frame
  std::vector<cv::Point3f> cam_pts;
  std::vector<cv::Point2f> img_pts;
  CloudToPoints3(pcl_cloud_in, &cam_pts);
  cv::Mat zeros = cv::Mat::zeros(1, 3, CV_64FC1);
  cv::projectPoints(cam_pts, zeros, zeros, model.fullIntrinsicMatrix(),
                    model.distortionCoeffs(), img_pts);
  /*
  cv::Mat disp(image.clone());
  for (const auto &p : img_pts) {
    cv::circle(disp, p, 1, cv::Scalar(255, 0, 0), 1);
  }
  cv::imshow("image", disp);
  cv::waitKey(1);
  */

  // Convert grayscale image to color
  cv::Mat color;
  if (image.channels() == 3) {
    color = image;
  } else if (image.channels() == 1) {
    cv::cvtColor(image, color, CV_GRAY2BGR);
  }
  // Construct a new point cloud, super hacky, will need to refactor this shit
  PointCloud<PointXYZRGB> pcl_cloud_out;
  pcl::copyPointCloud(pcl_cloud_in, pcl_cloud_out);
  pcl_cloud_out.points.clear();
  for (size_t i = 0; i < pcl_cloud_in.points.size(); ++i) {
    cv::Point2i pixel_rounded(img_pts[i].x, img_pts[i].y);
    if (IsInsideImage(pixel_rounded, image.cols, image.rows)) {
      PointXYZRGB point;
      point.x = pcl_cloud_in.points[i].x;
      point.y = pcl_cloud_in.points[i].y;
      point.z = pcl_cloud_in.points[i].z;
      const uchar *p = image.ptr<uchar>(pixel_rounded.y);
      const int col = pixel_rounded.x;
      point.r = p[3 * col + 2];
      point.g = p[3 * col + 1];
      point.b = p[3 * col + 0];
      pcl_cloud_out.points.push_back(point);
    }
  }
  // Overwrite width with new point size
  pcl_cloud_out.width = pcl_cloud_out.points.size();
  pcl::toROSMsg(pcl_cloud_out, *ros_cloud_out);
}

}  // namespace thermal_map
}  // namespace galt
