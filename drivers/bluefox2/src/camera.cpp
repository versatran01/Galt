#include "bluefox2/camera.h"

#define btoa(x) ((x) ? "true" : "false")
#define TIMEOUT 300

namespace bluefox2 {

Camera::Camera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh)
    : node_(comm_nh), pnode_(param_nh), ok_(false)
{
  readSettings();
  printSettings();

  camera_info_manager_ = boost::shared_ptr<CamInfoMgr>(
      new CamInfoMgr(pnode_, "bluefox2", calibration_url_));

  if (camera_info_manager_->isCalibrated()) {
    ROS_INFO("Camera has loaded calibration file");
  }

  // Set up image transport publisher
  image_transport::ImageTransport it(pnode_);
  camera_pub_ = it.advertiseCamera("image_raw", 1);

  // Count cameras
  device_count_ = device_manager_.deviceCount();
  ROS_INFO("Camera count: %d", device_count_);

  // Initialize camera
  if((id_ = findCameraSerial()) >= 0) {  // Found a camera
    if (initCamera()) {  // Initialized a camera
      ok_ = true;
      ROS_INFO("Camera %s initialized", serial_.c_str());
    } else {
      ROS_INFO("Camera %s initialization failed", serial_.c_str());
    }
  } else {
    ROS_WARN("No camera found.");
  }
}

Camera::~Camera()
{
  func_interface_->imageRequestReset(0, 0);
  device_manager_[id_]->close();
  ok_ = false;
}

void Camera::readSettings()
{
  pnode_.param("use_color", use_color_, false);
  pnode_.param("use_hdr", use_hdr_, false);
  pnode_.param("has_hdr", has_hdr_, false);
  pnode_.param("use_inverted_", use_inverted_, false);
  pnode_.param("use_binning_", use_binning_, false);
  pnode_.param("use_auto_exposure", use_auto_exposure_, false);
  pnode_.param("exposure_time_us", exposure_time_us_, 10000);
  pnode_.param("height", height_, 480);
  pnode_.param("width", width_, 752);
  pnode_.param("fps", fps_, 30.0);
  pnode_.param("gain", gain_, 0.0);
  pnode_.param<std::string>("frame_id", frame_id_, "bluefox2");
  pnode_.param<std::string>("mode", mode_, "standalone");  // requires a hint
  if (!pnode_.getParam("serial", serial_)) {
    serial_ = std::string("");
    ROS_ERROR("No camera serial provided.");
  }
  if (!pnode_.getParam("calibration_url", calibration_url_)) {
    calibration_url_ = "";
    ROS_WARN("No calibration file specified.");
  }
}

void Camera::printSettings()
{
  ROS_INFO("Input settings:");
  ROS_INFO("Serial:   %s", serial_.c_str());
  ROS_INFO("Mode:     %s", mode_.c_str());
  ROS_INFO("Calib:    %s", calibration_url_.c_str());
  ROS_INFO("Frame:    %s", frame_id_.c_str());
  ROS_INFO("Height:   %d", height_);
  ROS_INFO("Width:    %d", width_);
  ROS_INFO("FPS:      %f", fps_);
  ROS_INFO("Gain:     %f", gain_);
  ROS_INFO("Color:    %s", btoa(use_color_));
  ROS_INFO("Auto exp: %s", btoa(use_auto_exposure_));
  ROS_INFO("Binning:  %s", btoa(use_binning_));
  ROS_INFO("Inverted: %s", btoa(use_inverted_));
  ROS_INFO("HDR:      %s", btoa(use_hdr_));
}

int Camera::findCameraSerial()
{
  if (device_count_ > 0) {
    for (int k = 0; k < device_count_; ++k) {
      if (device_manager_[k]->serial.read() == serial_) {
        return k;
      }
    }
  }
  return -1;
}

void Camera::printMvErrorMsg(const mvAcquireException &e,
                             const std::string header) {
  std::cout << header << " " << device_manager_[id_]->serial.read()
            << "(error code: " << e.getErrorCode() << "("
            << e.getErrorCodeAsString() << "))." << std::endl
            << "Press [Enter] to end the application..." << std::endl;
  PRESS_A_KEY;
}

bool Camera::ok()
{
  return ok_;
}

bool Camera::initCamera()
{
  ROS_INFO("Initializing camera: %s(%s)",
           device_manager_[id_]->family.read().c_str(),
           device_manager_[id_]->serial.read().c_str());

  try {
    device_manager_[id_]->open();
  } catch (const mvAcquireException &e) {
    printMvErrorMsg(e, "An error occurred while openning the device");
    return false;
  }

  try {
    func_interface_ =
        new mvIMPACT::acquire::FunctionInterface(device_manager_[id_]);
  } catch (const mvAcquireException &e) {
    printMvErrorMsg(e,
                     "An error occured while creating the function interface");
    return false;
  }

  try {
    statistics_ = new mvIMPACT::acquire::Statistics(device_manager_[id_]);
  } catch (const mvAcquireException &e) {
    printMvErrorMsg(
        e, "An error occurred while initializing the statistical information");
    return false;
  }

  // Using the "Base" settings (default)
  mvIMPACT::acquire::SettingsBlueFOX settings(device_manager_[id_]);
  ROS_INFO("Setting camera...");

  // Height
  if (height_ > 0) {
    settings.cameraSetting.aoiHeight.write(height_);
    ROS_WARN("Height: %d", height_);
  }

  // Width
  if (width_ > 0) {
    settings.cameraSetting.aoiWidth.write(width_);
    ROS_WARN("Width: %d", width_);
  }

  // Binning
  if (use_binning_) {
    settings.cameraSetting.binningMode.write(cbmBinningHV);
    ROS_WARN("2X Binning");
  } else {
    ROS_WARN("No Binning");
  }

  // Gain
  settings.cameraSetting.autoGainControl.write(agcOff);
  if (gain_ >= 0.0) {
    settings.cameraSetting.gain_dB.write(gain_);
    ROS_WARN("Gain: %f", gain_);
  } else {
    settings.cameraSetting.autoGainControl.write(agcOn);
    ROS_WARN("Auto Gain");
  }

  // Color
  if (use_color_) {
    settings.imageDestination.pixelFormat.write(idpfRGB888Packed);
    ROS_WARN("Color Images");
  } else {
    settings.imageDestination.pixelFormat.write(idpfRaw);
    ROS_WARN("Grayscale/Bayer Images");
  }

  // Auto exposure
  // modified controller for better results, be careful about the minimum exposure time
  if (use_auto_exposure_) {
    settings.cameraSetting.autoControlParameters.controllerSpeed.write(acsUserDefined);
    settings.cameraSetting.autoControlParameters.controllerGain.write(0.5);
    settings.cameraSetting.autoControlParameters.controllerIntegralTime_ms.write(100);
    settings.cameraSetting.autoControlParameters.controllerDerivativeTime_ms.write(0.0001);
    settings.cameraSetting.autoControlParameters.desiredAverageGreyValue.write(100);
    settings.cameraSetting.autoControlParameters.controllerDelay_Images.write(0);
    settings.cameraSetting.autoControlParameters.exposeLowerLimit_us.write(50);
    settings.cameraSetting.autoControlParameters.exposeUpperLimit_us.write(exposure_time_us_);
    settings.cameraSetting.autoExposeControl.write(aecOn);
    ROS_WARN("Auto Exposure w/ Max Exposure Time (us): %d",
             settings.cameraSetting.autoControlParameters.exposeUpperLimit_us
                 .read());
  } else {
    settings.cameraSetting.expose_us.write(exposure_time_us_);
    ROS_WARN("Exposure Time (us): %d",
             settings.cameraSetting.expose_us.read());
  }

  // HDR
  if (has_hdr_) {
    if (use_hdr_) {
      settings.cameraSetting.getHDRControl().HDRMode.write(cHDRmFixed0);
      settings.cameraSetting.getHDRControl().HDREnable.write(bTrue);
      ROS_WARN("Enable HDR ...");
      ROS_WARN("KneePoint 0:");
      ROS_WARN("  Voltage (mV):      %d", settings.cameraSetting.getHDRControl()
                                              .getHDRKneePoint(0)
                                              .HDRControlVoltage_mV.read());
      ROS_WARN("  Parts per Million: %d", settings.cameraSetting.getHDRControl()
                                              .getHDRKneePoint(0)
                                              .HDRExposure_ppm.read());
      ROS_WARN("KneePoint 1:");
      ROS_WARN("  Voltage (mV):      %d", settings.cameraSetting.getHDRControl()
                                              .getHDRKneePoint(1)
                                              .HDRControlVoltage_mV.read());
      ROS_WARN("  Parts per Million: %d", settings.cameraSetting.getHDRControl()
                                              .getHDRKneePoint(1)
                                              .HDRExposure_ppm.read());
    } else {
      settings.cameraSetting.getHDRControl().HDREnable.write(bFalse);
      ROS_WARN("HDR Off");
    }
  } else {
    ROS_WARN("No HDR");
  }

  // prefill the capture queue. There can be more then 1 queue for some device,
  // but only one for now
  mvIMPACT::acquire::SystemSettings sys_settings(device_manager_[id_]);
  sys_settings.requestCount.write(1);

  // Mode
  if (mode_ == "master") {
    ROS_WARN("Set Master Camera");
    // settings.cameraSetting.triggerMode.write(ctmOnDemand);
    settings.cameraSetting.flashMode.write(cfmDigout0);
    settings.cameraSetting.flashType.write(cftStandard);
    settings.cameraSetting.flashToExposeDelay_us.write(0);
  } else if (mode_ == "slave") {
    ROS_WARN("Set Slave Camera");
    settings.cameraSetting.triggerMode.write(ctmOnHighLevel);
    settings.cameraSetting.triggerSource.write(ctsDigIn0);
    settings.cameraSetting.frameDelay_us.write(0);
  } else {
    ROS_WARN("Standalone Camera");
  }

  return true;
}

bool Camera::grabImage(sensor_msgs::ImagePtr image)
{
  const unsigned char *image_frame = NULL;
  bool status   = false;

  // Request and wait for image
  func_interface_->imageRequestSingle();
  usleep(10000);  // necessary short sleep to warm up the camera

  int requestNr = INVALID_ID;
  requestNr = func_interface_->imageRequestWaitFor(TIMEOUT);

  // Got image
  if (func_interface_->isRequestNrValid(requestNr)) {
    pRequest_ = func_interface_->getRequest(requestNr);
    if (pRequest_->isOK()) {
      // Set image properties
      int channels  = pRequest_->imageChannelCount.read();
      image->height = pRequest_->imageHeight.read();
      image->width = pRequest_->imageWidth.read();
      image->step = pRequest_->imageChannelCount.read() *
                    pRequest_->imageWidth.read();
      if (channels == 1) {
        image->encoding = sensor_msgs::image_encodings::MONO8;
      } else if (channels == 3) {
        image->encoding = sensor_msgs::image_encodings::BGR8;
      }
      // Resize image only when necessary
      if (image->data.size() != image->step * image->height)
        image->data.resize(image->step * image->height);
      // Copy data
      image_frame = (const unsigned char *)pRequest_->imageData.read();
      if (use_inverted_) {
        std::reverse_copy(image_frame,
                          image_frame + image->width * image->height * channels,
                          &image->data[0]);
      } else {
        memcpy(&image->data[0], image_frame,
               image->width * image->height * channels);
      }
      // Release capture request
      func_interface_->imageRequestUnlock(requestNr);
      // Set image header
      image->header.stamp = ros::Time::now();
      image->header.frame_id = std::string(frame_id_);
      status = true;
    } else {
      ROS_ERROR("Invalid Image");
      // Clear all image received and reset capture
      func_interface_->imageRequestUnlock(requestNr);
      status = false;
    }
  } else {
    ROS_ERROR("Invalid Image Request");
    // Clear all image received and reset capture
    if (func_interface_->isRequestNrValid(requestNr)) {
      pRequest_ = func_interface_->getRequest(requestNr);
      func_interface_->imageRequestUnlock(requestNr);
    }
    status = false;
  }
  return status;
}

void Camera::feedImage()
{
  ros::Rate ros_rate(fps_);

  sensor_msgs::CameraInfoPtr camera_info(
      new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
  sensor_msgs::ImagePtr image(new sensor_msgs::Image);

  while (pnode_.ok()) {
      if (grabImage(image)) {
        camera_pub_.publish(image, camera_info);
      }
    }
    ros_rate.sleep();
    ros::spinOnce();
  }
}
