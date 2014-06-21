#include "bluefox2/bluefox2.h"

namespace bluefox2 {

Camera::Camera(string serial, mv_params_s mv_params)
    : ok_(false),
      serial_(serial),
      params_(mv_params),
      device_(NULL),
      func_interface_(NULL),
      stats_(NULL),
      request_(NULL),
      bf_settings_(NULL),
      sys_settings_(NULL) {
  // Locate device
  if ((device_count_ = device_manager_.deviceCount()) > 0) {
    if ((id_ = findDeviceId()) >= 0) {
      device_ = device_manager_[id_];
      label_ = string("\033[0;34m[BLFOX] [") + serial_ + string("]:\033[0m ");
      DISP("Deivce count:", device_count_);
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

  bf_settings_ = new SettingsBlueFOX(device_);
  sys_settings_ = new SystemSettings(device_);

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
  if (bf_settings_ != NULL) {
    delete bf_settings_;
  }
  if (sys_settings_ != NULL) {
    delete sys_settings_;
  }
  ok_ = false;
}

void Camera::init(bool verbose) {
  if (!(ok_ = open())) {
    throw std::runtime_error("Failed to open device");
  }
  DISP("Initializing camera: ",
       device_->family.read() + " / " + device_->serial.read());

  applySettings();

  if (verbose) {
    printSettings();
    printDetails();
  }
}

void Camera::applySettings() {
  // prefill the capture queue
  sys_settings_->requestCount.write(1);

  // AOI
  setAoi(params_.width, params_.height);

  // Binning
  if (params_.binning) {
    bf_settings_->cameraSetting.binningMode.write(cbmBinningHV);
  }

  // Color
  if (params_.color) {
    bf_settings_->imageDestination.pixelFormat.write(idpfRGB888Packed);
    if (params_.white_balance == "indoor") {
      bf_settings_->imageProcessing.whiteBalance.write(wbpFluorescent);
    } else if (params_.white_balance == "outdoor") {
      bf_settings_->imageProcessing.whiteBalance.write(wbpDayLight);
    }
  } else {
    bf_settings_->imageDestination.pixelFormat.write(idpfRaw);
  }

  // Gain
  bf_settings_->cameraSetting.autoGainControl.write(agcOff);
  if (params_.gain >= 0.0) {
    bf_settings_->cameraSetting.gain_dB.write(params_.gain);
  } else {
    bf_settings_->cameraSetting.autoGainControl.write(agcOn);
  }

  // Expose
  if (params_.auto_expose) {
    bf_settings_->cameraSetting.autoControlParameters.controllerSpeed.write(
        acsUserDefined);
    bf_settings_->cameraSetting.autoControlParameters.controllerGain.write(0.5);
    bf_settings_->cameraSetting.autoControlParameters.controllerIntegralTime_ms
        .write(100);
    bf_settings_->cameraSetting.autoControlParameters
        .controllerDerivativeTime_ms.write(0.0001);
    bf_settings_->cameraSetting.autoControlParameters.desiredAverageGreyValue
        .write(100);
    bf_settings_->cameraSetting.autoControlParameters.controllerDelay_Images
        .write(0);
    bf_settings_->cameraSetting.autoControlParameters.exposeLowerLimit_us.write(
        50);
    bf_settings_->cameraSetting.autoControlParameters.exposeUpperLimit_us.write(
        params_.expose_us);
    bf_settings_->cameraSetting.autoExposeControl.write(aecOn);
  } else {
    bf_settings_->cameraSetting.expose_us.write(params_.expose_us);
  }

  // Mode
  if (params_.mode == "master") {
    // settings.cameraSetting.triggerMode.write(ctmOnDemand);
    bf_settings_->cameraSetting.flashMode.write(cfmDigout0);
    bf_settings_->cameraSetting.flashType.write(cftStandard);
    bf_settings_->cameraSetting.flashToExposeDelay_us.write(0);
  } else if (params_.mode == "slave") {
    bf_settings_->cameraSetting.triggerMode.write(ctmOnHighLevel);
    bf_settings_->cameraSetting.triggerSource.write(ctsDigIn0);
    bf_settings_->cameraSetting.frameDelay_us.write(0);
  }

  // TODO: how to detect if camera supports hdr mode
  // HDR
  // std::cout << "hdr" << std::endl;
  // if (params_.hdr) {
  //   bf_settings_->cameraSetting.getHDRControl().HDRMode.write(cHDRmFixed0);
  //   bf_settings_->cameraSetting.getHDRControl().HDREnable.write(bTrue);
  //   std::cout << "Enable HDR ..." << std::endl;
  //   std::cout << "KneePoint 0:" << std::endl;
  //   std::cout << "  Voltage (mv):      "
  //             << bf_settings_->cameraSetting.getHDRControl()
  //                    .getHDRKneePoint(0)
  //                    .HDRControlVoltage_mV.read() << std::endl;
  //   std::cout << "  Parts per Million: "
  //             << bf_settings_->cameraSetting.getHDRControl()
  //                    .getHDRKneePoint(0)
  //                    .HDRExposure_ppm.read() << std::endl;
  //   std::cout << "KneePoint 1:" << std::endl;
  //   std::cout << "  Voltage (mv):      "
  //             << bf_settings_->cameraSetting.getHDRControl()
  //                    .getHDRKneePoint(1)
  //                    .HDRControlVoltage_mV.read() << std::endl;
  //   std::cout << "  Parts per Million: "
  //             << bf_settings_->cameraSetting.getHDRControl()
  //                    .getHDRKneePoint(1)
  //                    .HDRExposure_ppm.read() << std::endl;
  // } else {
  //   bf_settings_->cameraSetting.getHDRControl().HDREnable.write(bFalse);
  // }
}

void Camera::setAoi(const int &w, const int &h) {
  int w_max = bf_settings_->cameraSetting.aoiWidth.getMaxValue();
  int w_min = bf_settings_->cameraSetting.aoiWidth.getMinValue();
  int h_max = bf_settings_->cameraSetting.aoiHeight.getMaxValue();
  int h_min = bf_settings_->cameraSetting.aoiHeight.getMinValue();

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
  bf_settings_->cameraSetting.aoiStartX.write(x);
  bf_settings_->cameraSetting.aoiStartY.write(y);
  bf_settings_->cameraSetting.aoiWidth.write(w);
  bf_settings_->cameraSetting.aoiHeight.write(h);
}

bool Camera::grabImage(mv_image_s &mv_image) {
  bool status = false;

  // Request and wait for image
  func_interface_->imageRequestSingle();
  // usleep(100);  // necessary short sleep to warm up the camera

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
      DISP("Invalid image", "");
      // Clear all image received and reset capture
      func_interface_->imageRequestUnlock(requestNr);
      status = false;
    }
  } else {
    DISP("Invalid image request", "");
    // Clear all image received and reset capture
    if (func_interface_->isRequestNrValid(requestNr)) {
      request_ = func_interface_->getRequest(requestNr);
      func_interface_->imageRequestUnlock(requestNr);
    }
    status = false;
  }

  return status;
}

void Camera::printDetails() const {
  DISP("Detials for camera:  ", serial_);
  DISP("--Id:                ", device_->deviceID.readS());
  DISP("--Product:           ", device_->product.readS());
  DISP("--Family:            ", device_->family.readS());
  DISP("--Class:             ", device_->deviceClass.readS());
  DISP("--Serial:            ", device_->serial.readS());
  DISP("--State:             ", device_->state.readS());
  DISP("--In use:            ", btoa(device_->isInUse()));
  DISP("--Is open:           ", btoa(device_->isOpen()));
}

void Camera::printSettings() const {
  DISP("Settings for camera: ", serial_);
  DISP("--Width:             ", params_.width);
  DISP("--Height:            ", params_.height);
  DISP("--FPS:               ", params_.fps);
  DISP("--Mode:              ", params_.mode);
  DISP("--Gain:              ", params_.gain);
  DISP("--Color:             ", btoa(params_.color));
  DISP("--Inverted:          ", btoa(params_.inverted));
  DISP("--Binning:           ", btoa(params_.binning));
  DISP("--Auto Expose:       ", btoa(params_.auto_expose));
  DISP("--Expose Time:       ", params_.expose_us);
  DISP("--Balance:           ", params_.white_balance);
}

void Camera::printStats() const {
  DISP("Stats for camera:    ", serial_);
  DISP("--FPS:               ", stats_->framesPerSecond.read());
  DISP("--Capture time:      ", stats_->captureTime_s.read());
  DISP("--Process time:      ", stats_->imageProcTime_s.read());
}

void Camera::printMvErrorMsg(const ImpactAcquireException &e,
                             const std::string msg) const {
  std::cout << label_ << msg << " " << device_->serial.read()
            << "(error code: " << e.getErrorCode() << "("
            << e.getErrorCodeAsString() << "))." << std::endl
            << "Press [Enter] to end the application..." << std::endl;
}

}  // namepace bluefox2
