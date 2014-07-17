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

#include <PvSystem.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvBuffer.h>
#include <PvPipeline.h>


namespace flir_gige {

struct FreeDevice {
  void operator() (PvDevice *device) const {
    PvDevice::Free(device);
  }
};

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

  void FindDevice();
  void ConnectDevice();
  void OpenStream();
  void ConfigureStream();
  void CreatePipeline();
  void AcquireImages();

 private:
//  typedef std::unique_ptr<PvDevice, decltype(PvDevice::Free)*> PvDeviceUPtr;
//  typedef std::unique_ptr<PvStream, decltype(PvStream::Free)*> PvStreamUPtr;
  typedef std::unique_ptr<PvDevice, FreeDevice> PvDevicePtr;
  typedef std::unique_ptr<PvStream, FreeStream> PvStreamPtr;
  typedef std::unique_ptr<PvPipeline> PvPipelinePtr;

  std::string ip_address_;
  PvSystem system_;
  const PvDeviceInfo *device_info_;
  PvDevicePtr device_;
  PvStreamPtr stream_;
  PvPipelinePtr pipeline_;
};  // class GigeCamera

}  // namespace flir_gige
