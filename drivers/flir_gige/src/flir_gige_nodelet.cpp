/*
 * flir_gige_nodelet.cpp
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of flir_gige.
 *
 *	Created on: 21/08/2014
 */

#include "flir_gige/flir_gige.h"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace flir_gige {

class FlirNodelet : public nodelet::Nodelet {
 public:
  FlirNodelet() : nodelet::Nodelet() {}
  ~FlirNodelet() { flir_gige_->End(); }

  virtual void onInit() {
    try {
      flir_gige_.reset(new FlirGige(getPrivateNodeHandle()));
      flir_gige_->Run();
    }
    catch (const std::exception &e) {
      ROS_ERROR_STREAM("flir_gige: " << e.what());
    }
  }

 private:
  std::unique_ptr<FlirGige> flir_gige_;

};

PLUGINLIB_DECLARE_CLASS(flir_gige, FlirNodelet, flir_gige::FlirNodelet,
                        nodelet::Nodelet)

}  // namespace flir_gige
