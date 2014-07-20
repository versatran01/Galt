/*
 * gige_camera.h
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  Created on: 12/7/2014
 *      Author: Chao Qu
 */
#ifndef FLIR_GIGE_GIGE_CAMERA_H_
#define FLIR_GIGE_GIGE_CAMERA_H_

#include <stdint.h>

#include <string>
#include <memory>
#include <thread>
#include <functional>

#include <PvSystem.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvBuffer.h>
#include <PvPipeline.h>

#include <opencv2/core/core.hpp>

namespace flir_gige {
// Digital output big
enum BitSize { BIT8BIT = 2, BIT14BIT };

// Config struct
struct GigeConfig {
  bool color{false};
  int width{320};
  int height{256};
  int bit{2};
};

// Functor for free PvDevice
struct FreeDevice {
  void operator()(PvDevice *device) const { PvDevice::Free(device); }
};

// Functor for free PvStream
struct FreeStream {
  void operator()(PvStream *stream) const { PvStream::Free(stream); }
};

class GigeCamera {
 public:
  GigeCamera(const std::string &ip_address);
  GigeCamera(const GigeCamera &) = delete;             // No copy constructor
  GigeCamera &operator=(const GigeCamera &) = delete;  // No assignment operator

  // Find and connect to device, create PvDevice, PvStream and PvPipeline
  void Connect();
  // Configure the camera before image acquisition
  void Configure(const GigeConfig &config);
  // Start pipeline, enable stream and start acquisition
  void Start();
  // Stop acquisition, disable stream and stop pipeline
  void Stop();
  // Release all resources we hold (PvDevice, PvStream and PvPipleline)
  void Disconnect();
  // Return Acquisition status
  const bool IsAcquire() const { return acquire_; }

  std::function<void(const cv::Mat &image)> use_image;

 private:
  void FindDevice(const std::string &ip);
  void ConnectDevice();
  void OpenStream();
  void ConfigureStream();
  void CreatePipeline();
  void StartAcquisition();
  void StopAcquisition();
  void AcquireImages();
  double GetSpotTemperature(double pixel, double R, double F, double B,
                            double O);

  void SetAoi(int width, int height);
  void SetPixelFormat(BitSize bit);

  typedef std::unique_ptr<PvDevice, FreeDevice> PvDevicePtr;
  typedef std::unique_ptr<PvStream, FreeStream> PvStreamPtr;
  typedef std::unique_ptr<PvPipeline> PvPipelinePtr;
  typedef std::unique_ptr<std::thread> ThreadPtr;
  typedef std::unique_ptr<cv::Mat> MatPtr;

  bool raw_{false};
  bool acquire_{false};
  bool color_{false};  // false - grayscale, true - jet
  std::string label_{"\033[0;35m[ FLIR]:\033[0m "};

  PvSystem system_;
  const PvDeviceInfo *dinfo_;
  PvDevicePtr device_;
  PvStreamPtr stream_;
  PvPipelinePtr pipeline_;
  ThreadPtr image_thread_;
  MatPtr image_raw_;

};  // class GigeCamera

}  // namespace flir_gige

#endif  // FLIR_GIGE_GIGE_CAMERA_H_
