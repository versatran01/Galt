#include "bluefox2/camera.h"

#define btoa(x) ((x) ? "true" : "false")
#define TIMEOUT 300  // ms

namespace bluefox2 {

Camera::Camera(ros::NodeHandle param_nh)
    : pnode_(param_nh), ok_(false), calibrated_(false) {
  // Read and display settings from launch file
  readSettings();
  printSettings();

  // Count cameras
  device_count_ = device_manager_.deviceCount();
  ROS_INFO("Camera count: %d", device_count_);

  // Initialize camera
  if ((id_ = findCameraSerial()) >= 0) {  // Found a camera
    if (initCamera()) {                   // Initialized a camera
      // Set up image transport publisher and camera info manager
      image_transport::ImageTransport it(pnode_);
      camera_pub_ = it.advertiseCamera("image_raw", 1);
      camera_info_manager_ = CameraInfoManagerPtr(
          new CameraInfoManager(pnode_, "bluefox2", calibration_url_));
      calibrated_ = checkCameraInfo();
      ok_ = true;
      ROS_INFO("Camera %s initialized", serial_.c_str());
    } else {
      ROS_INFO("Camera %s initialization failed", serial_.c_str());
    }
  } else {
    ROS_WARN("No specified camera found.");
  }
}

Camera::~Camera() {
  if (ok_) {
    func_interface_->imageRequestReset(0, 0);
    device_manager_[id_]->close();
    ok_ = false;
  }
}

void Camera::readSettings() {
  // Required
  if (!pnode_.getParam("serial", serial_)) {
    serial_ = std::string("");
    ROS_ERROR("No camera serial provided.");
  }

  // Optional
  pnode_.param("use_color", use_color_, false);
  pnode_.param("use_hdr", use_hdr_, false);
  pnode_.param("has_hdr", has_hdr_, false);
  pnode_.param("use_inverted_", use_inverted_, false);
  pnode_.param("use_binning_", use_binning_, false);
  pnode_.param("use_auto_exposure", use_auto_exposure_, false);
  pnode_.param("exposure_time_us", exposure_time_us_, 10000);
  pnode_.param("height", height_, 480);
  pnode_.param("width", width_, 752);
  pnode_.param("fps", fps_, 20.0);
  pnode_.param("gain", gain_, 0.0);
  pnode_.param<std::string>("frame_id", frame_id_, "bluefox2");
  pnode_.param<std::string>("mode", mode_, "standalone");  // requires a hint
  pnode_.param<std::string>("white_balance", white_balance_, "none");
  if (!pnode_.getParam("calibration_url", calibration_url_)) {
    calibration_url_ = "";
    ROS_WARN("No calibration file specified.");
  }
}

void Camera::printSettings() const {
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
  ROS_INFO("WBP:      %s", white_balance_.c_str());  // White balance parameter
  ROS_INFO("Auto exp: %s", btoa(use_auto_exposure_));
  ROS_INFO("Binning:  %s", btoa(use_binning_));
  ROS_INFO("Inverted: %s", btoa(use_inverted_));
  ROS_INFO("HDR:      %s", btoa(use_hdr_));
}

bool Camera::checkCameraInfo() const {
  sensor_msgs::CameraInfoPtr camera_info(
      new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
  // Chech height, width and camera matrix
  if (camera_info->width != (unsigned int)width_ ||
      camera_info->height != (unsigned int)height_) {
    ROS_WARN("bluefox2: Calibration dimension mismatch.");
    return false;
  }
  if (!camera_info_manager_->isCalibrated()) {
    ROS_WARN("bluefox2: Camera not calibrated.");
    return false;
  }
  return true;
}

int Camera::findCameraSerial() const {
  if (device_count_ > 0) {
    for (int k = 0; k < device_count_; ++k) {
      if (device_manager_[k]->serial.read() == serial_) {
        return k;
      }
    }
  }
  return -1;
}

void Camera::printMvErrorMsg(const ImpactAcquireException &e,
                             const std::string header) const {
  std::cout << header << " " << device_manager_[id_]->serial.read()
            << "(error code: " << e.getErrorCode() << "("
            << e.getErrorCodeAsString() << "))." << std::endl
            << "Press [Enter] to end the application..." << std::endl;
  PRESS_A_KEY;
}

bool Camera::ok() const { return ok_; }

int Camera::fps() const { return fps_; }

bool Camera::initCamera() {
  ROS_INFO("Initializing camera: %s(%s)",
           device_manager_[id_]->family.read().c_str(),
           device_manager_[id_]->serial.read().c_str());

  try {
    device_manager_[id_]->open();
  }
  catch (const ImpactAcquireException &e) {
    printMvErrorMsg(e, "An error occurred while openning the device");
    return false;
  }

  try {
    func_interface_ = new FunctionInterface(device_manager_[id_]);
  }
  catch (const ImpactAcquireException &e) {
    printMvErrorMsg(e, "An error occured while creating function interface");
    return false;
  }

  try {
    statistics_ = new Statistics(device_manager_[id_]);
  }
  catch (const ImpactAcquireException &e) {
    printMvErrorMsg(e, "An error occurred while initializing statistical info");
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
    if (white_balance_ == "indoor") {
      settings.imageProcessing.whiteBalance.write(wbpFluorescent);
    } else if (white_balance_ == "outdoor") {
      settings.imageProcessing.whiteBalance.write(wbpDayLight);
    }
    ROS_WARN("White Balance: %s", white_balance_.c_str());
    ROS_WARN("Color Images");
  } else {
    settings.imageDestination.pixelFormat.write(idpfRaw);
    ROS_WARN("Grayscale/Bayer Images");
  }

  // Auto exposure
  // modified controller for better results, be careful about the minimum
  // exposure time
  if (use_auto_exposure_) {
    settings.cameraSetting.autoControlParameters.controllerSpeed.write(
        acsUserDefined);
    settings.cameraSetting.autoControlParameters.controllerGain.write(0.5);
    settings.cameraSetting.autoControlParameters.controllerIntegralTime_ms
        .write(100);
    settings.cameraSetting.autoControlParameters.controllerDerivativeTime_ms
        .write(0.0001);
    settings.cameraSetting.autoControlParameters.desiredAverageGreyValue.write(
        100);
    settings.cameraSetting.autoControlParameters.controllerDelay_Images.write(
        0);
    settings.cameraSetting.autoControlParameters.exposeLowerLimit_us.write(50);
    settings.cameraSetting.autoControlParameters.exposeUpperLimit_us.write(
        exposure_time_us_);
    settings.cameraSetting.autoExposeControl.write(aecOn);
    ROS_WARN("Auto Exposure w/ Max Exposure Time (us): %d",
             settings.cameraSetting.autoControlParameters.exposeUpperLimit_us
                 .read());
  } else {
    settings.cameraSetting.expose_us.write(exposure_time_us_);
    ROS_WARN("Exposure Time (us): %d", settings.cameraSetting.expose_us.read());
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

bool Camera::grabImage(sensor_msgs::ImagePtr image) {
  bool status = false;

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
      int channels = pRequest_->imageChannelCount.read();
      image->height = pRequest_->imageHeight.read();
      image->width = pRequest_->imageWidth.read();
      image->step =
          pRequest_->imageChannelCount.read() * pRequest_->imageWidth.read();
      if (channels == 1) {
        image->encoding = sensor_msgs::image_encodings::MONO8;
      } else if (channels == 3) {
        image->encoding = sensor_msgs::image_encodings::BGR8;
      }
      // Resize image only when necessary
      if (image->data.size() != image->step * image->height) {
        image->data.resize(image->step * image->height);
      }
      // Copy data
      const unsigned char *image_frame = NULL;
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
      image->header.frame_id = frame_id_;
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

void Camera::feedImage() {
  sensor_msgs::CameraInfoPtr camera_info(
      new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
  sensor_msgs::ImagePtr image(new sensor_msgs::Image);

  if (grabImage(image)) {
    // Only publish height and width if camera not calibrated
    if (!calibrated_) {
      camera_info.reset(new sensor_msgs::CameraInfo());
      camera_info->width = image->width;
      camera_info->height = image->height;
    }
    camera_info->header.stamp = image->header.stamp;
    camera_info->header.frame_id = image->header.frame_id;
    camera_pub_.publish(image, camera_info);
  }
}

}  // namepace bluefox2
