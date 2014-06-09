#include <iostream>
#include <algorithm>
#include <string>

#include <errno.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

#include <boost/shared_ptr.hpp>

#define PRESS_A_KEY getchar();

using camera_info_manager::CameraInfoManager;
using mvIMPACT::acquire::ImpactAcquireException;

typedef boost::shared_ptr<CameraInfoManager> CameraInfoManagerPtr;

namespace bluefox2 {

class Camera {
  public:
    /**
     * @brief Constructor
     *
     * @param comm_nh ros node handle
     * @param param_nh private ros node handle
     */
    Camera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);

    /**
     * @brief Destructor
     */
    ~Camera();

    /**
     * @brief True if camera is initialized
     */
    bool ok() const;

    /**
     * @brief Accessor for fps
     * @return Camera fps
     */
    int fps() const;

    /**
     * @brief Publish image
     */
    void feedImage();

  private:
    // Node handle
    ros::NodeHandle node_;
    ros::NodeHandle pnode_;
    // mvIMPACT Acquire device manager
    mvIMPACT::acquire::DeviceManager device_manager_;
    // create an interface to the device found
    mvIMPACT::acquire::FunctionInterface *func_interface_;
    // establish access to the statistic properties
    mvIMPACT::acquire::Statistics *statistics_;
    // Image request
    const mvIMPACT::acquire::Request *pRequest_;

    bool ok_;
    int device_count_;
    bool calibrated_;

    // Settings
    bool use_color_;
    bool use_hdr_;
    bool has_hdr_;
    bool use_inverted_;
    bool use_binning_;
    bool use_auto_exposure_;
    int exposure_time_us_;
    int height_;
    int width_;
    double fps_;
    double gain_;

    int id_;
    std::string mode_;  ///< mode can be master, slave and standalone
    std::string serial_;  ///< serial number on the back of camera
    std::string frame_id_;
    std::string calibration_url_;

    CameraInfoManagerPtr camera_info_manager_;
    image_transport::CameraPublisher camera_pub_;

    /**
     * @brief Find camera index by serial number
     * @return camera index in device manager, -1 if not found
     */
    int findCameraSerial() const;

    /**
     * @brief Read settings from launch file
     */
    void readSettings();

    /**
     * @brief Print all bluefox setings
     */
    void printSettings() const;

    /**
     * @brief Check calibration status
     * @return True if calibration file is valid
     */
    bool checkCameraInfo() const;

    /**
     * @brief Print mv error message
     *
     * @param e mvIMPACT acquire exception
     * @param header Message header
     */
    void printMvErrorMsg(const ImpactAcquireException &e,
                         const std::string header) const;

    /**
     * @brief Initialize camera
     * @return ture if successful
     */
    bool initCamera();

    /**
     * @brief Get image from camera
     *
     * @param image Pointer to ros sensor_msgs::Image
     * @return true if successful
     */
    bool grabImage(sensor_msgs::ImagePtr image);

};  // class Camera

}  // namespace bluefox2
