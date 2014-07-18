#include "flir_gige/flir_nodelet.h"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(flir_gige, FlirNodelet, flir_gige::FlirNodelet,
                        nodelet::Nodelet)

namespace flir_gige {
FlirNodelet::FlirNodelet() : nodelet::Nodelet() {}

FlirNodelet::~FlirNodelet() {
  flir_thread_->join();
}

void FlirNodelet::onInit() {
  flir_gige_.reset(new FlirGige(getPrivateNodeHandle()));
  flir_thread_.reset(new std::thread(&FlirGige::Run, flir_gige_.get()));
}

}  // namespace flir_gige
