#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>

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

 public:
  FlirNode(const ros::NodeHandle& nh) : nh_{nh}, it_{nh} {
    image_pub_ = it_.advertise("image_raw", 1);
  }
};

}  // namespace flir_gige

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "flir_node");
  ros::NodeHandle nh("~");
  flir_gige::FlirNode fn(nh);
  ros::spin();
  return 0;
}
