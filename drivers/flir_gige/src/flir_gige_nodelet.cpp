#include "flir_gige/flir_nodelet.h"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(flir_gige, FlirNodelet, flir_gige::FlirNodelet,
                        nodelet::Nodelet)

namespace flir_gige {
FlirNodelet::FlirNodelet() : nodelet::Nodelet() {}

FlirNodelet::~FlirNodelet() {
  flir_gige_->End();
}

void FlirNodelet::onInit() {
  flir_gige_.reset(new FlirGige(getPrivateNodeHandle()));
  flir_gige_->Run();
}

}  // namespace flir_gige
