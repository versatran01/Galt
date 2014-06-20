#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include "bluefox2/bluefox2.h"

using namespace sensor_msgs;

using std::string;
using bluefox2::mv_params_t;
using bluefox2::mv_image_t;
using camera_info_manager::CameraInfoManager;
typedef boost::shared_ptr<CameraInfoManager> CameraInfoManagerPtr;

image_transport::CameraPublisher camera_pub;
unsigned int seq = 0;

// Read parameters from launch file
bluefox2::mv_params_t ReadParams(ros::NodeHandle nh) {
  bluefox2::mv_params_t mv_params;

  nh.param<bool>("color", mv_params.color, false);
  nh.param<bool>("hdr", mv_params.hdr, false);
  nh.param<bool>("inverted", mv_params.inverted, false);
  nh.param<bool>("binning", mv_params.binning, false);
  nh.param<bool>("auto_expose", mv_params.auto_expose, false);
  nh.param<int>("expose_us", mv_params.expose_us, 10000);
  nh.param<int>("height", mv_params.height, 480);
  nh.param<int>("width", mv_params.width, 752);
  nh.param<int>("fps", mv_params.fps, 20);
  nh.param<double>("gain", mv_params.gain, 0.0);
  nh.param<string>("mode", mv_params.mode, "standalone");
  nh.param<string>("white_balance", mv_params.white_balance, "default");

  return mv_params;
}

// Wrapper function for publishing camera
void publishCamera(mv_image_t &mv_image, const CameraInfoPtr &camera_info,
                   const string &frame_id) {
  ImagePtr image(new Image);

  image->header.stamp = ros::Time::now();
  image->header.frame_id = frame_id;
  image->header.seq = seq++;

  image->width = mv_image.width;
  image->step = mv_image.step;
  image->height =  mv_image.height;

  if (mv_image.channel == 1) {
    image->encoding = image_encodings::MONO8;
  } else if (mv_image.channel == 3) {
    image->encoding = image_encodings::BGR8;
  }
  std::swap(image->data, mv_image.data);

  camera_info->header = image->header;
  camera_pub.publish(image, camera_info);
}

// Main
int main(int argc, char **argv) {
  ros::init(argc, argv, "bluefox2");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  // Get camera serial number
  string serial;
  nh.param<string>("serial", serial, "");

  // Set node information
  string topic("image_raw");
  string node = string("cam_") + serial;

  // Get settings from launch file
  mv_params_t mv_params = ReadParams(nh);

  // Get other ros related settings
  string calibration_url;
  string frame_id;
  nh.param<string>("calibration_url", calibration_url, "");
  nh.param<string>("frame_id", frame_id, "camera");

  try {
    // Instantiate and initialize camera
    bluefox2::Camera camera(serial, mv_params);
    ROS_INFO("Device count: %d", camera.device_count());
    camera.init(true);  // true - print settings

    // Publish images
    if (camera.ok()) {
      ros::Rate loop_rate(camera.fps());
      ROS_INFO("Publishing image topic to /%s/%s", node.c_str(), topic.c_str());

      // Set up camera publisher and check camera calibration
      camera_pub = it.advertiseCamera("image_raw", 1);
      CameraInfoManagerPtr camera_info_manager = CameraInfoManagerPtr(
          new CameraInfoManager(nh, "bluefox2", calibration_url));
      CameraInfoPtr camera_info(
          new CameraInfo(camera_info_manager->getCameraInfo()));
      if (!camera_info_manager->isCalibrated() ||
          camera_info->width != static_cast<unsigned int>(mv_params.width) ||
          camera_info->height != static_cast<unsigned int>(mv_params.height)) {
        // Only set dimension if calibration file mismatch
        camera_info.reset(new sensor_msgs::CameraInfo());
        camera_info->width = mv_params.width;
        camera_info->height = mv_params.height;
        ROS_WARN("bluefox2: Camera not calibrated");
      }

      // Continuous grab and publish image
      mv_image_t mv_image;
      while (ros::ok()) {
        // Grab image from camera
        camera.grabImage(mv_image);
        publishCamera(mv_image, camera_info, frame_id);
        if (seq % (10 * camera.fps()) == 0) {
          camera.printStats();
        }
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
  }
  catch (const std::exception &e) {
    ROS_ERROR("bluefox2: %s", e.what());
  }

  return 0;
}
