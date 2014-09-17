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

// Implement this later
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
  static const std::string camera_frame = "mv_stereo/left";
  if (!GetLatestTransfrom(camera_frame, "world", &tf_stamped)) {
    return;
  }

  // Convert ros PointCloud2 to pcl PointCloud<T>
  const sensor_msgs::PointCloud2 &cloud_ros_w = srv.response.cloud;
  PointCloud<PointXYZ> cloud_pcl_c;
  TransformCloud(cloud_ros_w, tf_stamped.transform, camera_frame, &cloud_pcl_c);

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

/*
void Img2pclNode::ProjectCloud(const sensor_msgs::PointCloud &cloud_in,
                               const cv::Mat &image,
                               const image_geometry::PinholeCameraModel &model,
                               sensor_msgs::PointCloud &cloud_out) const {
  std::vector<cv::Point3f> camera_points;
  CloudToPoints(cloud_in, camera_points);
  std::vector<cv::Point2f> image_points;
  cv::Mat zeros = cv::Mat::zeros(1, 3, CV_64FC1);
  cv::projectPoints(camera_points, zeros, zeros, model.fullIntrinsicMatrix(),
                    model.distortionCoeffs(), image_points);
  PixelsToCloud(image, image_points, cloud_in, cloud_out);
}
*/

/*
void Img2pclNode::PixelsToCloud(const cv::Mat &image,
                                const std::vector<cv::Point2f> &pixels,
                                const sensor_msgs::PointCloud cloud_in,
                                sensor_msgs::PointCloud &cloud_out) const {
  cloud_out.header = cloud_in.header;
  sensor_msgs::ChannelFloat32 channel;
  channel.name = "rgb";
  union {
    uint8_t rgb[4];
    float val;
  } color;

  // Go through each pixel, add to cloud_out if it's inside the image
  for (decltype(pixels.size()) i = 0, e = pixels.size(); i != e; ++i) {
    const cv::Point2f &pixel = pixels[i];
    cv::Point2i pixel_rounded(pixel.x, pixel.y);
    if (InsideImage(image.size(), pixel_rounded)) {
      cloud_out.points.push_back(cloud_in.points[i]);
      // Get rgb data from image
      const uchar *p = image.ptr<uchar>(pixel_rounded.y);
      int col = pixel.x;
      color.rgb[0] = p[3 * col];
      color.rgb[1] = p[3 * col + 1];
      color.rgb[2] = p[3 * col + 2];
      color.rgb[3] = 0;
      channel.values.push_back(color.val);
    }
  }
  cloud_out.channels.push_back(channel);
}
*/

/*
void Img2pclNode::CloudToPoints(const sensor_msgs::PointCloud &cloud,
                                std::vector<cv::Point3f> &points) const {
  for (const geometry_msgs::Point32 &point : cloud.points) {
    points.push_back(cv::Point3f(point.x, point.y, point.z));
  }
}
*/

}  // namespace thermal_map
}  // namespace galt
