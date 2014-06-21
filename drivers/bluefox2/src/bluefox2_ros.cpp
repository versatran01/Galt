#include "bluefox2/bluefox2.h"
#include "bluefox2/bluefox2_ros.h"

namespace bluefox2 {

CameraRos::CameraRos(const ros::NodeHandle &nh) : nh_(nh), seq_(0) {
  // Get camera serial number
  string serial;
  nh_.param<string>("serial", serial, "");
  nh.param<string>("calibration_url", calibration_url_, "");
  nh.param<string>("frame_id", frame_id_, "camera");
  // Set node information
  node_ = string("cam_") + serial;
  // Get settings from launch file
  mv_params_s mv_params = readParams();
  // Create and initialize camera
  camera_ = CameraPtr(new Camera(serial, mv_params));
}

CameraRos::~CameraRos() {};

void CameraRos::init() {
  camera_->init(true);
  // Set up camera publisher
  image_transport::ImageTransport it(nh_);
  camera_pub_ = it.advertiseCamera("image_raw", 1);
  camera_info_manager_ = CameraInfoManagerPtr(
      new CameraInfoManager(nh_, "bluefox2", calibration_url_));
  camera_info_ =
      CameraInfoPtr(new CameraInfo(camera_info_manager_->getCameraInfo()));
  // Check camera calibration
  if (!camera_info_manager_->isCalibrated() ||
      camera_info_->width != static_cast<unsigned int>(camera_->width()) ||
      camera_info_->height != static_cast<unsigned int>(camera_->height())) {
    // Only set dimension if calibration file mismatch
    camera_info_.reset(new sensor_msgs::CameraInfo());
    camera_info_->width = camera_->width();
    camera_info_->height = camera_->height();
    ROS_WARN("bluefox2: Camera %s not calibrated", camera_->serial().c_str());
  }
}

void CameraRos::publish() {
  mv_image_s mv_image;
  ros::Rate loop_rate(camera_->fps());
  ROS_INFO("Publishing image topic to /%s/image_raw", node_.c_str());

  while (ros::ok()) {
    camera_->grabImage(mv_image);
    // Prepare messages to publish
    ImagePtr image(new Image);
    image->header.stamp = ros::Time::now();
    image->header.frame_id = frame_id_;
    image->header.seq = seq_++;
    image->width = mv_image.width;
    image->height = mv_image.height;
    image->step = mv_image.step;
    if (mv_image.channel == 1) {
      image->encoding = image_encodings::MONO8;
    } else if (mv_image.channel == 3) {
      image->encoding = image_encodings::BGR8;
    }
    std::swap(image->data, mv_image.data);

    camera_info_->header = image->header;
    camera_pub_.publish(image, camera_info_);

    if (seq_ == 100) {
      camera_->printStats();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

mv_params_s CameraRos::readParams() {
  mv_params_s mv_params;

  nh_.param<bool>("color", mv_params.color, false);
  nh_.param<bool>("hdr", mv_params.hdr, false);
  nh_.param<bool>("inverted", mv_params.inverted, false);
  nh_.param<bool>("binning", mv_params.binning, false);
  nh_.param<bool>("auto_expose", mv_params.auto_expose, false);
  nh_.param<int>("expose_us", mv_params.expose_us, 5000);
  nh_.param<int>("width", mv_params.width, 752);
  nh_.param<int>("height", mv_params.height, 480);
  nh_.param<double>("fps", mv_params.fps, 20.0);
  nh_.param<double>("gain", mv_params.gain, 0.0);
  nh_.param<string>("mode", mv_params.mode, "standalone");
  nh_.param<string>("white_balance", mv_params.white_balance, "default");

  return mv_params;
}

}  // namespace bluefox2
