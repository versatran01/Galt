/*
 * gige_camera.h
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  Created on: 12/7/2014
 *      Author: Chao Qu
 */

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

// Functor for free PvDevice
struct FreeDevice {
  void operator() (PvDevice *device) const {
    PvDevice::Free(device);
  }
};

// Functor for free PvStream
struct FreeStream {
  void operator() (PvStream *stream) const {
    PvStream::Free(stream);
  }
};

class GigeCamera {
 public:
  GigeCamera(const std::string &ip_address = "");
  GigeCamera(const GigeCamera&) = delete;  // No copy constructor
  GigeCamera& operator=(const GigeCamera&) = delete;  // No assignment operator

  // Find and connect to device, create PvDevice, PvStream and PvPipeline
  void Connect();
  //
  void Configure();
  //
  void Start();
  void Stop();
  void Disconnect();

  std::function<void(const cv::Mat &image)> use_image;

 private:
  void FindDevice();
  void ConnectDevice();
  void OpenStream();
  void ConfigureStream();
  void CreatePipeline();
  void AcquireImages();
  void ImageThread();

  typedef std::unique_ptr<PvDevice, FreeDevice> PvDevicePtr;
  typedef std::unique_ptr<PvStream, FreeStream> PvStreamPtr;
  typedef std::unique_ptr<PvPipeline> PvPipelinePtr;
  typedef std::unique_ptr<std::thread> ThreadPtr;

  std::string ip_address_;
  PvSystem system_;
  const PvDeviceInfo *device_info_;
  PvDevicePtr device_;
  PvStreamPtr stream_;
  PvPipelinePtr pipeline_;
  ThreadPtr image_thread_;
};  // class GigeCamera

}  // namespace flir_gige
