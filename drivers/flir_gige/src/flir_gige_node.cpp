#include <memory>
#include <stdexcept>

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
    // Setup image publisher and dynamic reconfigure callback
    image_pub_ = it_.advertise("image_raw", 1);
    server_.setCallback(
        boost::bind(&FlirNode::ReconfigureCallback, this, _1, _2));
  }

  FlirNode(const FlirNode &) = delete;             // No copy constructor
  FlirNode &operator=(const FlirNode &) = delete;  // No assignment operator

  bool init() {
    //    camera_->FindDevice();
    //    camera_->ConnectDevice();
    //    camera_->OpenStream();
    //    camera_->ConfigureStream();
    //    camera_->CreatePipeline();
    //    camera_->AcquireImages();
    return true;
  }

  void PublishImage(const cv::Mat &image) {
    if (!ros::ok()) {
      ros::shutdown();
    }
    // Construct a cv image
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr->header.stamp = ros::Time::now();
    cv_ptr->header.frame_id = frame_id_;
    cv_ptr->header.seq = seq++;
    cv_ptr->image = image;
    cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
    // Convert to ros image msg and publish
    image_msg_ = cv_ptr->toImageMsg();
    image_pub_.publish(image_msg_);
  }

  void ReconfigureCallback(flir_gige::FlirConfig &config, int level) {
    ROS_INFO("%s", "In reconfigure callback");
    ROS_INFO_STREAM("Level " << level << "Width " << config.width << " Height "
                             << config.height);
  }
};

}  // namespace flir_gige

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "flir_node");

  ros::NodeHandle nh("~");
  double fps = 0;
  nh.param<double>("fps", fps, 20);

  try {
    flir_gige::FlirNode flir_node(nh, fps);
    flir_node.init();
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR_STREAM(e.what());
  }

  return 0;
}
