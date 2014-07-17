#include <memory>
#include <stdexcept>
#include <functional>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include "flir_gige/gige_camera.h"
#include "flir_gige/FlirConfig.h"

namespace flir_gige {

class FlirNode {
 private:
  // ROS related
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  sensor_msgs::ImagePtr image_msg_;
  dynamic_reconfigure::Server<flir_gige::FlirConfig> server_;
  ros::Rate rate_;
  std::string frame_id_;
  unsigned seq = 0;
  // Flir Camera
  std::shared_ptr<flir_gige::GigeCamera> camera_;

 public:
  FlirNode(const ros::NodeHandle &nh, const double fps)
      : nh_{nh}, it_{nh}, rate_{fps} {
    nh_.param<std::string>("frame_id", frame_id_, std::string("flir"));
    // Create a camera
    std::string ip_address;
    nh_.param<std::string>("ip_address", ip_address, std::string(""));
    camera_ = std::make_shared<flir_gige::GigeCamera>(ip_address);
    camera_->use_image = std::bind(&FlirNode::PublishImage, this,
                                   std::placeholders::_1);
    // Setup image publisher and dynamic reconfigure callback
    image_pub_ = it_.advertise("image_raw", 1);
    server_.setCallback(
        boost::bind(&FlirNode::ReconfigureCallback, this, _1, _2));
  }

  FlirNode(const FlirNode &) = delete;             // No copy constructor
  FlirNode &operator=(const FlirNode &) = delete;  // No assignment operator

  void Init() {
    camera_->Connect();
    camera_->Configure();
    camera_->Start();
  }

  void PublishImage(const cv::Mat &image) {
    if (!ros::ok()) {
      ros::shutdown();
    }
    // Construct a cv image
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage());
    cv_ptr->header.stamp = ros::Time::now();
    cv_ptr->header.frame_id = frame_id_;
    cv_ptr->header.seq = seq++;
    cv_ptr->image = image;
    cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
    // Convert to ros image msg and publish
    image_msg_ = cv_ptr->toImageMsg();
    image_pub_.publish(image_msg_);
    rate_.sleep();
  }

  void ReconfigureCallback(flir_gige::FlirConfig &config, int level) {
    // Do nothing when first starting
    if (level < 0) {
      ROS_INFO("Starting reconfigure server.");
      return;
    }

    // Stop the camera if in acquisition
    if (camera_->IsAcquire()) {
      //Stop the image thread
      camera_->Stop();
      std::cout << "Stop camera" << std::endl;
      camera_->Disconnect();
      std::cout << "Disconnect camera" << std::endl;
    }
    // Reconnect the camera
    camera_->Connect();
    // Reconfigure the camera
    camera_->Configure();
    // Restart the camera
    camera_->Start();
  }

};  // class FlirNode

}  // namespace flir_gige

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "flir_node");

  ros::NodeHandle nh("~");
  double fps = 0;
  nh.param<double>("fps", fps, 20);

  try {
    flir_gige::FlirNode flir_node(nh, fps);
    flir_node.Init();
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR_STREAM(e.what());
  }

  return 0;
}
