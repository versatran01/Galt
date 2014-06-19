#include "bluefox2/bluefox2.h"

#define btoa(x) ((x) ? "true" : "false")

namespace bluefox2 {

Camera::Camera(string serial, mv_params_t mv_params)
    : ok_(false),
      serial_(serial),
      params_(mv_params),
      device_(NULL),
      func_interface_(NULL),
      stats_(NULL),
      request_(NULL),
      bluefox_settings_(NULL),
      system_settings_(NULL) {
  // Count cameras
  device_count_ = device_manager_.deviceCount();

  if (device_count_ > 0) {
    if ((id_ = findDeviceId()) >= 0) {
      device_ = device_manager_[id_];
      std::cout << "Initializing camera" << device_->family.read() << "/"
                << device_->serial.read() << std::endl;
    } else {
      throw std::runtime_error("No device with serial number: " + serial_);
    }
  } else {
    throw std::runtime_error("No devices found.");
  }
}

Camera::~Camera() { close(); }

int Camera::findDeviceId() const {
  for (int k = 0; k < device_count_; ++k) {
    if (device_manager_[k]->serial.read() == serial_) {
      return k;
    }
  }
  return -1;
}

bool Camera::open() {
  try {
    device_->open();
  }
  catch (const ImpactAcquireException &e) {
    printMvErrorMsg(e, "Failed to open the device");
    close();
    return false;
  }

  try {
    func_interface_ = new FunctionInterface(device_);
  }
  catch (const ImpactAcquireException &e) {
    printMvErrorMsg(e, "Failed to create a function interface");
    close();
    return false;
  }

  try {
    stats_ = new Statistics(device_);
  }
  catch (const ImpactAcquireException &e) {
    printMvErrorMsg(e, "Fialed to initialize statistical info");
    close();
    return false;
  }

  bluefox_settings_ = new SettingsBlueFOX(device_);
  system_settings_ = new SystemSettings(device_);

  return true;
}

void Camera::close() {
  if (device_ != NULL) {
    device_->close();
  }
  if (func_interface_ != NULL) {
    func_interface_->imageRequestReset(0, 0);
    delete func_interface_;
  }
  if (stats_ != NULL) {
    delete stats_;
  }
  if (request_ != NULL) {
    delete request_;
  }
  if (bluefox_settings_ != NULL) {
    delete bluefox_settings_;
  }
  if (system_settings_ != NULL) {
    delete system_settings_;
  }
  ok_ = false;
}

void Camera::init(bool verbose) {
  if (!(ok_ = open())) {
    throw std::runtime_error("Failed to open device");
  }

  applySettings();

  if (verbose) {
    printSettings();
  }
}

void Camera::applySettings() {
  // AOI
  setAoi(params_.width, params_.height);

  // Binning
  if (params_.binning) {
    bluefox_settings_->cameraSetting.binningMode.write(cbmBinningHV);
  }

  // Color
  if (params_.color) {
  }

  // Gain

  // Exposure

  // Mode
}

void Camera::printSettings() const {}

void Camera::printStats() const {}

void Camera::printMvErrorMsg(const ImpactAcquireException &e,
                             const std::string header) const {
  std::cout << header << " " << device_manager_[id_]->serial.read()
            << "(error code: " << e.getErrorCode() << "("
            << e.getErrorCodeAsString() << "))." << std::endl
            << "Press [Enter] to end the application..." << std::endl;
  PRESS_A_KEY;
}

void Camera::setAoi(const int &w, const int &h) {
  int w_max = bluefox_settings_->cameraSetting.aoiWidth.getMaxValue();
  int w_min = bluefox_settings_->cameraSetting.aoiWidth.getMinValue();
  int h_max = bluefox_settings_->cameraSetting.aoiHeight.getMaxValue();
  int h_min = bluefox_settings_->cameraSetting.aoiHeight.getMinValue();

  if (w > w_max || w < w_min || h > h_max || h < h_min) {
    // Invalid settings of width and height
    std::stringstream error_msg;
    error_msg << "Invalid dimension, width: " << w << " out of [" << w_min
              << "," << w_max << "], height: " << h << " out of [" << h_min
              << "," << h_max << "]";
    throw std::runtime_error(error_msg.str());
  }
  // Set StartX and StartY so that the image is centered
  int x = (w_max - w) / 2;
  int y = (h_max - h) / 2;
  bluefox_settings_->cameraSetting.aoiStartX.write(x);
  bluefox_settings_->cameraSetting.aoiStartY.write(y);
  bluefox_settings_->cameraSetting.aoiWidth.write(w);
  bluefox_settings_->cameraSetting.aoiHeight.write(h);
}
/*
bool Camera::initCamera() {
  ROS_INFO("Initializing camera: %s(%s)",
           device_manager_[id_]->family.read().c_str(),
           device_manager_[id_]->serial.read().c_str());


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
*/

/*
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
*/

/*
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
*/

}  // namepace bluefox2
