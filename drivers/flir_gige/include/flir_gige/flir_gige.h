#ifndef FLIR_GIGE_FLIR_GIGE_H_
#define FLIR_GIGE_FLIR_GIGE_H_

#include <memory>
#include <utility>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include "flir_gige/gige_camera.h"
#include "flir_gige/FlirConfig.h"

namespace flir_gige {

class FlirGige {
 private:
  // ROS related
  ros::NodeHandle nh_;
  std::string frame_id_;
  std::unique_ptr<ros::Rate> rate_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher camera_pub_;
  sensor_msgs::ImagePtr image_;
  sensor_msgs::CameraInfoPtr cinfo_;
  ros::Publisher temp_pub_;
  dynamic_reconfigure::Server<FlirConfig> server_;
  // Flir Camera
  std::unique_ptr<GigeCamera> camera_;

 public:
  FlirGige(const ros::NodeHandle &nh);
  FlirGige(const FlirGige &) = delete;             // No copy constructor
  FlirGige &operator=(const FlirGige &) = delete;  // No assignment operator

  void Run();
  void End();
  void PublishImage(const cv::Mat &image, const std::vector<double> &planck);
  void PublishTemperature(const std::pair<double, double> &spot);
  std::string GetImageEncoding(const cv::Mat &image);
  void ReconfigureCallback(FlirConfig &config, int level);

};  // class FlirGige

}  // namespace flir_gige

#endif  // FLIR_GIGE_FLIR_GIGE_H_
