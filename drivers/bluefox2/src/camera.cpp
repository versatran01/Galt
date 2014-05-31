#include "bluefox2/camera.h"

namespace bluefox2 {

Camera::Camera(ros::NodeHandle _comm_nh, ros::NodeHandle _param_nh) : node(_comm_nh), pnode(_param_nh)
{
  pnode.param("use_stereo", use_stereo, false);
  pnode.param("use_split_image", use_split_image, false);
  pnode.param("use_color", use_color, false);
  pnode.param("use_hdr", use_hdr, false);
  pnode.param("has_hdr", has_hdr, false);
  pnode.param("use_inverted", use_inverted, false);
  pnode.param("use_binning", use_binning, false);
  pnode.param("use_auto_exposure", use_auto_exposure, false);
  pnode.param("fps", fps, 30.0);
  pnode.param("gain", gain, 0.0);
  pnode.param("exposure_time_us", exposure_time_us, 10000);
  pnode.param("serial_left",      serial0, std::string(""));
  pnode.param("serial_right",     serial1, std::string(""));
  ok = false;

  pub  = pnode.advertise<sensor_msgs::Image>("image", 10);
  if (use_stereo && use_split_image) {
    publ = pnode.advertise<sensor_msgs::Image>("left",  10);
    pubr = pnode.advertise<sensor_msgs::Image>("right", 10);
  }

  // Count cameras
  devCnt = devMgr.deviceCount();
  ROS_WARN("Camera Cnt:  %d", devCnt);

  // Init cameras
  if (use_stereo && devCnt >= 2 && devCnt <= 10 && serial0 != serial1) {
    int cnt = 0;
    for (unsigned int k = 0; k < devCnt; k++) {
      if (devMgr[k]->serial.read() == serial0) {
        id0 = k;
        cnt++;
      }
      if (devMgr[k]->serial.read() == serial1) {
        id1 = k;
        cnt++;
      }
    }
    if (cnt == 2 && initSingleMVDevice(id0) && initSingleMVDevice(id1))
      ok = true;
  } else if (!use_stereo && devCnt >= 1 && devCnt <= 10) {
    int cnt = 0;
    for (unsigned int k = 0; k < devCnt; k++) {
      if (devMgr[k]->serial.read() == serial0) {
        id0 = k;
        cnt++;
      }
    }
    if (cnt == 1 && initSingleMVDevice(id0))
      ok = true;
  }
  if (!ok)
    ROS_ERROR("Camera Init Failed.");
}


Camera::~Camera()
{
  if (use_stereo) {
    fi[id0]->imageRequestReset(0, 0);
    devMgr[id0]->close();
    fi[id1]->imageRequestReset(0, 0);
    devMgr[id1]->close();
  } else {
    fi[id0]->imageRequestReset(0, 0);
    devMgr[id0]->close();
  }
  ok = false;
}


bool Camera::isOK()
{
  return ok;
}


bool Camera::initSingleMVDevice(unsigned int id)
{
  ROS_WARN("Camera Found:  %s(%s)", devMgr[id]->family.read().c_str(), devMgr[id]->serial.read().c_str());

  try {
    devMgr[id]->open();
  } catch (const mvIMPACT::acquire::ImpactAcquireException &e) {
    std::cout << "An error occurred while opening the device " << devMgr[id]->serial.read()
              << "(error code: " << e.getErrorCode() << "(" << e.getErrorCodeAsString() << "))."
              << std::endl << "Press [ENTER] to end the application..." << std::endl;
    return false;
  }

  try {
    fi[id] = new mvIMPACT::acquire::FunctionInterface(devMgr[id]);
  } catch (const mvIMPACT::acquire::ImpactAcquireException &e) {
    std::cout << "An error occurred while creating the function interface on device " << devMgr[id]->serial.read()
              << "(error code: " << e.getErrorCode() << "(" << e.getErrorCodeAsString() << "))."
              << std::endl << "Press [ENTER] to end the application..." << std::endl;
    return false;
  }

  try {
    statistics[id] = new mvIMPACT::acquire::Statistics(devMgr[id]);
  } catch (const mvIMPACT::acquire::ImpactAcquireException &e) {
    std::cout << "An error occurred while initializing the statistical information on device " << devMgr[id]->serial.read()
              << "(error code: " << e.getErrorCode() << "(" << e.getErrorCodeAsString() << "))."
              << std::endl << "Press [ENTER] to end the application..." << std::endl;
    return false;
  }

  // Set Properties
  mvIMPACT::acquire::SettingsBlueFOX settings(devMgr[id]); // Using the "Base" settings (default)

  // Binning
  if (use_binning) {
    settings.cameraSetting.binningMode.write(cbmBinningHV);
    ROS_WARN("2X Binning");
  } else {
    ROS_WARN("No Binning");
  }

  // Gain
  settings.cameraSetting.autoGainControl.write(agcOff);
  if (gain >= 0.0) {
    settings.cameraSetting.gain_dB.write(gain);
    ROS_WARN("Gain:  %f", gain);
  } else {
    settings.cameraSetting.autoGainControl.write(agcOn);
    ROS_WARN("Auto Gain");
  }

  // Auto exposure, modified controller for better results, be careful about the minimum exposure time
  if (use_auto_exposure) {
    settings.cameraSetting.autoControlParameters.controllerSpeed.write(acsUserDefined);
    settings.cameraSetting.autoControlParameters.controllerGain.write(0.5);
    settings.cameraSetting.autoControlParameters.controllerIntegralTime_ms.write(100);
    settings.cameraSetting.autoControlParameters.controllerDerivativeTime_ms.write(0.0001);
    settings.cameraSetting.autoControlParameters.desiredAverageGreyValue.write(100);
    settings.cameraSetting.autoControlParameters.controllerDelay_Images.write(0);
    settings.cameraSetting.autoControlParameters.exposeLowerLimit_us.write(50);
    settings.cameraSetting.autoControlParameters.exposeUpperLimit_us.write(exposure_time_us);
    settings.cameraSetting.autoExposeControl.write(aecOn);
    ROS_WARN("Auto Exposure w/ Max Exposure Time (us) :  %d", settings.cameraSetting.autoControlParameters.exposeUpperLimit_us.read());
  } else {
    settings.cameraSetting.expose_us.write(exposure_time_us);
    ROS_WARN("Exposure Time (us) :  %d", settings.cameraSetting.expose_us.read());
  }

  // HDR
  if (has_hdr) {
    if (use_hdr) {
      settings.cameraSetting.getHDRControl().HDRMode.write(cHDRmFixed0);
      settings.cameraSetting.getHDRControl().HDREnable.write(bTrue);
      ROS_WARN("Enable HDR ...");
      ROS_WARN("KneePoint 0:");
      ROS_WARN("  Voltage (mV):      %d", settings.cameraSetting.getHDRControl().getHDRKneePoint(0).HDRControlVoltage_mV.read());
      ROS_WARN("  Parts per Million: %d", settings.cameraSetting.getHDRControl().getHDRKneePoint(0).HDRExposure_ppm.read());
      ROS_WARN("KneePoint 1:");
      ROS_WARN("  Voltage (mV):      %d", settings.cameraSetting.getHDRControl().getHDRKneePoint(1).HDRControlVoltage_mV.read());
      ROS_WARN("  Parts per Million: %d", settings.cameraSetting.getHDRControl().getHDRKneePoint(1).HDRExposure_ppm.read());
    } else {
      settings.cameraSetting.getHDRControl().HDREnable.write(bFalse);
      ROS_WARN("HDR Off");
    }
  } else {
    ROS_WARN("No HDR");
  }

  // Color
  if (use_color) {
    // RGB image
    settings.imageDestination.pixelFormat.write(idpfRGB888Packed);
    ROS_WARN("Color Images");
  } else {
    // Raw image
    settings.imageDestination.pixelFormat.write(idpfRaw);
    ROS_WARN("Grayscale/Bayer Images");
  }

  // prefill the capture queue. There can be more then 1 queue for some device, but only one for now
  mvIMPACT::acquire::SystemSettings ss(devMgr[id]);
  ss.requestCount.write(1);

  // Only for stereo, skip if only one camera exists
  if (use_stereo) {
    if (id == id0) { // Master camera
      ROS_WARN("Set Master Camera\n");
      //settings.cameraSetting.triggerMode.write(ctmOnDemand);
      settings.cameraSetting.flashMode.write(cfmDigout0);
      settings.cameraSetting.flashType.write(cftStandard);
      settings.cameraSetting.flashToExposeDelay_us.write(0);
    } else {                                             // Slave camera
      ROS_WARN("Set Slave Camera\n");
      settings.cameraSetting.triggerMode.write(ctmOnHighLevel);
      settings.cameraSetting.triggerSource.write(ctsDigIn0);
      settings.cameraSetting.frameDelay_us.write(0);
    }
  }

  return true;
}


void Camera::feedImages()
{
  ros::Rate r(fps);
  sensor_msgs::ImagePtr image(new sensor_msgs::Image);
  sensor_msgs::ImagePtr left(new sensor_msgs::Image);
  sensor_msgs::ImagePtr right(new sensor_msgs::Image);
  while (pnode.ok()) {
    if (use_stereo) {
      if (grab_stereo(image, left, right)) {
        image->header.stamp = capture_time;
        image->header.frame_id = std::string("image");
        left->header  = image->header;
        right->header = image->header;
        pub.publish(*image);
        if (use_split_image) {
          publ.publish(*left);
          pubr.publish(*right);
        }
      }
    } else {
      if (grab_monocular(image)) {
        image->header.stamp = capture_time;
        image->header.frame_id = std::string("image");
        pub.publish(*image);
      }
    }
    r.sleep();
    ros::spinOnce();
  }
}


bool Camera::grab_monocular(sensor_msgs::ImagePtr image)
{
  const unsigned char *img_frame = NULL;
  bool status   = false;
  // Request and wait for image
  fi[id0]->imageRequestSingle();
  capture_time = ros::Time::now();
  usleep(10000);  // necessary short sleep to warm up the camera

  int requestNr = INVALID_ID;
  requestNr = fi[id0]->imageRequestWaitFor(300);

  // Got image
  if (fi[id0]->isRequestNrValid(requestNr)) {
    pRequest[id0] = fi[id0]->getRequest(requestNr);
    if (pRequest[id0]->isOK()) {
      // Set image properties
      int channels  = pRequest[id0]->imageChannelCount.read();
      image->height = pRequest[id0]->imageHeight.read();
      image->width  = pRequest[id0]->imageWidth.read();
      image->step   = pRequest[id0]->imageChannelCount.read() * pRequest[id0]->imageWidth.read();
      if (channels == 1)
        image->encoding = sensor_msgs::image_encodings::MONO8;
      else if (channels == 3)
        image->encoding = sensor_msgs::image_encodings::BGR8;
      // Resize image only when necessary
      if (image->data.size() != image->step * image->height)
        image->data.resize(image->step * image->height);
      // Copy data
      img_frame = (const unsigned char *)pRequest[id0]->imageData.read();
      if (use_inverted)
        std::reverse_copy(img_frame, img_frame + image->width * image->height * channels, &image->data[0]);
      else
        memcpy(&image->data[0], img_frame, image->width * image->height * channels);
      // Release capture request
      fi[id0]->imageRequestUnlock(requestNr);
      status = true;
    } else {
      ROS_ERROR("Invalid Image");
      // Clear all image received and reset capture
      fi[id0]->imageRequestUnlock(requestNr);
      status = false;
    }
  } else {
    ROS_ERROR("Invalid Image Request");
    // Clear all image received and reset capture
    if (fi[id0]->isRequestNrValid(requestNr)) {
      pRequest[id0] = fi[id0]->getRequest(requestNr);
      fi[id0]->imageRequestUnlock(requestNr);
    }
    status = false;
  }
  return status;
}


bool Camera::grab_stereo(sensor_msgs::ImagePtr image, sensor_msgs::ImagePtr left, sensor_msgs::ImagePtr right)
{
  const unsigned char *img_frame = NULL;
  bool status   = false;

  // Request images from both cameras
  fi[id1]->imageRequestSingle();
  usleep(10000);  // necessary short sleep to warm up the camera
  fi[id0]->imageRequestSingle();
  capture_time = ros::Time::now();

  int requestNr[10] = {INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID};
  requestNr[id0] = fi[id0]->imageRequestWaitFor(300);
  requestNr[id1] = fi[id1]->imageRequestWaitFor(300);

  if (fi[id0]->isRequestNrValid(requestNr[id0]) && fi[id1]->isRequestNrValid(requestNr[id1])) {
    pRequest[id0] = fi[id0]->getRequest(requestNr[id0]);
    pRequest[id1] = fi[id1]->getRequest(requestNr[id1]);
    if (pRequest[id0]->isOK()                   && pRequest[id1]->isOK()                   &&
        pRequest[id0]->imageChannelCount.read() == pRequest[id1]->imageChannelCount.read() &&
        pRequest[id0]->imageHeight.read()       == pRequest[id1]->imageHeight.read()       &&
        pRequest[id0]->imageWidth.read()        == pRequest[id1]->imageWidth.read()) {
      // Set image properties
      int channels  = pRequest[id0]->imageChannelCount.read();
      image->height = pRequest[id0]->imageHeight.read() * 2;
      image->width  = pRequest[id0]->imageWidth.read();
      image->step   = pRequest[id0]->imageChannelCount.read() * pRequest[id0]->imageWidth.read();
      if (use_split_image) {
        left->height  = pRequest[id0]->imageHeight.read();
        left->width   = pRequest[id0]->imageWidth.read();
        left->step    = pRequest[id0]->imageChannelCount.read() * pRequest[id0]->imageWidth.read();
        right->height = pRequest[id0]->imageHeight.read();
        right->width  = pRequest[id0]->imageWidth.read();
        right->step   = pRequest[id0]->imageChannelCount.read() * pRequest[id0]->imageWidth.read();
      }
      if (channels == 1) {
        image->encoding = sensor_msgs::image_encodings::MONO8;
        if (use_split_image) {
          left->encoding  = sensor_msgs::image_encodings::MONO8;
          right->encoding = sensor_msgs::image_encodings::MONO8;
        }
      } else if (channels == 3) {
        image->encoding = sensor_msgs::image_encodings::BGR8;
        if (use_split_image) {
          left->encoding  = sensor_msgs::image_encodings::BGR8;
          right->encoding = sensor_msgs::image_encodings::BGR8;
        }
      }
      // Resize image only when necessary
      if (image->data.size() != image->step * image->height) {
        image->data.resize(image->step * image->height);
      }
      if (use_split_image) {
        if (left->data.size() != left->step * left->height)
          left->data.resize(left->step * left->height);
        if (right->data.size() != right->step * right->height)
          right->data.resize(right->step * right->height);
      }
      // Copy data
      img_frame = (const unsigned char *)pRequest[id0]->imageData.read();
      if (use_inverted) {
        std::reverse_copy(img_frame, img_frame + image->width * image->height / 2 * channels, &image->data[0]);
        if (use_split_image)
          std::reverse_copy(img_frame, img_frame + left->width * left->height * channels, &left->data[0]);
      } else {
        memcpy(&image->data[0], img_frame, image->width * image->height / 2 * channels);
        if (use_split_image)
          memcpy(&left->data[0], img_frame, left->width * left->height * channels);
      }

      img_frame = (const unsigned char *)pRequest[id1]->imageData.read();
      if (use_inverted) {
        std::reverse_copy(img_frame, img_frame + image->width * image->height / 2 * channels, &image->data[image->width * image->height / 2 * channels]);
        if (use_split_image)
          std::reverse_copy(img_frame, img_frame + right->width * right->height * channels, &right->data[0]);
      } else {
        memcpy(&image->data[image->width * image->height / 2 * channels], img_frame, image->width * image->height / 2 * channels);
        if (use_split_image)
          memcpy(&right->data[0], img_frame, right->width * right->height * channels);
      }

      // Release capture request
      fi[id0]->imageRequestUnlock(requestNr[id0]);
      fi[id1]->imageRequestUnlock(requestNr[id1]);
      status = true;
    } else {
      ROS_ERROR("Invalid Image");
      // Clear all image received and reset capture
      fi[id0]->imageRequestUnlock(requestNr[id0]);
      fi[id1]->imageRequestUnlock(requestNr[id1]);
      status = false;
    }
  } else {
    ROS_ERROR("Invalid Image Request");
    // Clear all image received and reset capture
    if (fi[id0]->isRequestNrValid(requestNr[id0])) {
      pRequest[id0] = fi[id0]->getRequest(requestNr[id0]);
      fi[id0]->imageRequestUnlock(requestNr[id0]);
    }
    if (fi[id1]->isRequestNrValid(requestNr[id1])) {
      pRequest[id1] = fi[id1]->getRequest(requestNr[id1]);
      fi[id1]->imageRequestUnlock(requestNr[id1]);
    }
    status = false;
  }
  return status;
}

}

