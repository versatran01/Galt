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

    flir_gige_.reset(new FlirGige(getPrivateNodeHandle()));
    flir_gige_->Run();
  }

 private:
  std::unique_ptr<FlirGige> flir_gige_;

};  // class FlirNodelet

PLUGINLIB_DECLARE_CLASS(flir_gige, FlirNodelet, flir_gige::FlirNodelet,
                        nodelet::Nodelet)

}  // namespace flir_gige
