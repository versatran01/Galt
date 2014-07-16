/*
 * gige_camera.h
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  Created on: 12/7/2014
 *      Author: Chao Qu
 */

#include <PvSystem.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvBuffer.h>

namespace flir_gige {

class GigeCamera {
 public:
  GigeCamera(const std::string &ip_address = "");
  GigeCamera(const GigeCamera&) = delete;  // No copy constructor
  GigeCamera& operator=(const GigeCamera&) = delete;  // No assignment operator

  void FindDevice();


 private:
  std::string ip_address_;
  PvSystem system_;
  const PvDeviceInfo *device_info_;
};  // class GigeCamera

}  // namespace flir_gige
