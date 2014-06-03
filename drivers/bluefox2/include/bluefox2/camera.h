#include <iostream>
#include <algorithm>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <errno.h>

namespace bluefox2 {

class Camera {
  public:
    Camera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
    ~Camera();
    bool isOK();
    void feedImages();

  private:
    // Node handle
    ros::NodeHandle node_, pnode_;
    // mvIMPACT Acquire device manager
    mvIMPACT::acquire::DeviceManager devMgr;
    // create an interface to the device found
    mvIMPACT::acquire::FunctionInterface *fi[10];
    // establish access to the statistic properties
    mvIMPACT::acquire::Statistics *statistics[10];
    // Image request
    const mvIMPACT::acquire::Request *pRequest[10];
    // Internal parameters that cannot be changed
    bool ok;
    ros::Time capture_time;
    ros::Publisher pub;
    ros::Publisher publ;
    ros::Publisher pubr;
    unsigned int devCnt;

    // User specified parameters
    bool use_stereo;
    bool use_split_image;
    bool use_color;
    bool use_hdr;
    bool has_hdr;
    bool use_inverted;
    bool use_binning;
    bool use_auto_exposure;
    double fps;
    double gain;
    int exposure_time_us;
    unsigned int id0;
    unsigned int id1;
    std::string serial0;
    std::string serial1;

    std::string calibration_url_;
    std::string camera_name_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    image_transport::CameraPublisher camera_pub_;

    bool initSingleMVDevice(unsigned int id);
    bool grab_monocular(sensor_msgs::ImagePtr image);
    bool grab_stereo(sensor_msgs::ImagePtr image,
                     sensor_msgs::ImagePtr left,
                     sensor_msgs::ImagePtr right);
};

}

