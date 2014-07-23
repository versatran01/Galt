#include "flir_gige/calibrator.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "flir_calibrator");
  ros::NodeHandle nh("~");

  flir_gige::Calibrator calibrator(nh);
  ros::spin();
}
