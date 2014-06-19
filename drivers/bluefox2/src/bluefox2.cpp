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

  if ((device_count_ = device_manager_.deviceCount()) > 0) {
    if ((id_ = findDeviceId()) >= 0) {
      device_ = device_manager_[id_];
      std::cout << "Initializing camera: " << device_->family.read() << "/"
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
  // prefill the capture queue
  system_settings_->requestCount.write(1);

  // AOI
  setAoi(params_.width, params_.height);

  // Binning
  if (params_.binning) {
    bluefox_settings_->cameraSetting.binningMode.write(cbmBinningHV);
  }

  // Color
  if (params_.color) {
    bluefox_settings_->imageDestination.pixelFormat.write(idpfRGB888Packed);
    if (params_.white_balance == "indoor") {
      bluefox_settings_->imageProcessing.whiteBalance.write(wbpFluorescent);
    } else if (params_.white_balance == "outdoor") {
      bluefox_settings_->imageProcessing.whiteBalance.write(wbpDayLight);
    }
  } else {
    bluefox_settings_->imageDestination.pixelFormat.write(idpfRaw);
  }

  // Gain
  bluefox_settings_->cameraSetting.autoGainControl.write(agcOff);
  if (params_.gain >= 0.0) {
    bluefox_settings_->cameraSetting.gain_dB.write(params_.gain);
  } else {
    bluefox_settings_->cameraSetting.autoGainControl.write(agcOn);
  }

  // Expose
  if (params_.auto_expose) {
    bluefox_settings_->cameraSetting.autoControlParameters.controllerSpeed
        .write(acsUserDefined);
    bluefox_settings_->cameraSetting.autoControlParameters.controllerGain.write(
        0.5);
    bluefox_settings_->cameraSetting.autoControlParameters
        .controllerIntegralTime_ms.write(100);
    bluefox_settings_->cameraSetting.autoControlParameters
        .controllerDerivativeTime_ms.write(0.0001);
    bluefox_settings_->cameraSetting.autoControlParameters
        .desiredAverageGreyValue.write(100);
    bluefox_settings_->cameraSetting.autoControlParameters
        .controllerDelay_Images.write(0);
    bluefox_settings_->cameraSetting.autoControlParameters.exposeLowerLimit_us
        .write(50);
    bluefox_settings_->cameraSetting.autoControlParameters.exposeUpperLimit_us
        .write(params_.expose_us);
    bluefox_settings_->cameraSetting.autoExposeControl.write(aecOn);
  } else {
    bluefox_settings_->cameraSetting.expose_us.write(params_.expose_us);
  }

  // Mode
  if (params_.mode == "master") {
    // settings.cameraSetting.triggerMode.write(ctmOnDemand);
    bluefox_settings_->cameraSetting.flashMode.write(cfmDigout0);
    bluefox_settings_->cameraSetting.flashType.write(cftStandard);
    bluefox_settings_->cameraSetting.flashToExposeDelay_us.write(0);
  } else if (params_.mode == "slave") {
    bluefox_settings_->cameraSetting.triggerMode.write(ctmOnHighLevel);
    bluefox_settings_->cameraSetting.triggerSource.write(ctsDigIn0);
    bluefox_settings_->cameraSetting.frameDelay_us.write(0);
  }

  // TODO: how to detect if camera supports hdr mode
  // HDR
  // std::cout << "hdr" << std::endl;
  // if (params_.hdr) {
  //   bluefox_settings_->cameraSetting.getHDRControl().HDRMode.write(cHDRmFixed0);
  //   bluefox_settings_->cameraSetting.getHDRControl().HDREnable.write(bTrue);
  //   std::cout << "Enable HDR ..." << std::endl;
  //   std::cout << "KneePoint 0:" << std::endl;
  //   std::cout << "  Voltage (mv):      "
  //             << bluefox_settings_->cameraSetting.getHDRControl()
  //                    .getHDRKneePoint(0)
  //                    .HDRControlVoltage_mV.read() << std::endl;
  //   std::cout << "  Parts per Million: "
  //             << bluefox_settings_->cameraSetting.getHDRControl()
  //                    .getHDRKneePoint(0)
  //                    .HDRExposure_ppm.read() << std::endl;
  //   std::cout << "KneePoint 1:" << std::endl;
  //   std::cout << "  Voltage (mv):      "
  //             << bluefox_settings_->cameraSetting.getHDRControl()
  //                    .getHDRKneePoint(1)
  //                    .HDRControlVoltage_mV.read() << std::endl;
  //   std::cout << "  Parts per Million: "
  //             << bluefox_settings_->cameraSetting.getHDRControl()
  //                    .getHDRKneePoint(1)
  //                    .HDRExposure_ppm.read() << std::endl;
  // } else {
  //   bluefox_settings_->cameraSetting.getHDRControl().HDREnable.write(bFalse);
  // }
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

bool Camera::grabImage(mv_image_t &mv_image) {
  bool status = false;

  // Request and wait for image
  func_interface_->imageRequestSingle();
  usleep(10000);  // necessary short sleep to warm up the camera

  int requestNr = INVALID_ID;
  requestNr = func_interface_->imageRequestWaitFor(TIMEOUT_MS);

  // Got image
  if (func_interface_->isRequestNrValid(requestNr)) {
    request_ = func_interface_->getRequest(requestNr);
    if (request_->isOK()) {
      // Set image properties
      mv_image.channel = request_->imageChannelCount.read();
      mv_image.height = request_->imageHeight.read();
      mv_image.width = request_->imageWidth.read();
      mv_image.step = mv_image.channel * mv_image.width;

      // Resize image only when necessary
      if (mv_image.data.size() !=
          (unsigned int)(mv_image.step * mv_image.height)) {
        mv_image.data.resize(mv_image.step * mv_image.height);
      }

      // Copy data
      const unsigned char *frame = NULL;
      frame = (const unsigned char *)request_->imageData.read();
      if (params_.inverted) {
        std::reverse_copy(frame, frame + mv_image.step * mv_image.height,
                          &mv_image.data[0]);
      } else {
        memcpy(&mv_image.data[0], frame, mv_image.step * mv_image.height);
      }

      // Release capture request
      func_interface_->imageRequestUnlock(requestNr);
      status = true;
    } else {
      std::cout << "Invalid image" << std::endl;
      // Clear all image received and reset capture
      func_interface_->imageRequestUnlock(requestNr);
      status = false;
    }
  } else {
    std::cout << "Invalid image request" << std::endl;
    // Clear all image received and reset capture
    if (func_interface_->isRequestNrValid(requestNr)) {
      request_ = func_interface_->getRequest(requestNr);
      func_interface_->imageRequestUnlock(requestNr);
    }
    status = false;
  }

  return status;
}

}  // namepace bluefox2
