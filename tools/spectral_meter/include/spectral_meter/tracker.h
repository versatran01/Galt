#ifndef SPECTRAL_METER_TRACKER_H_
#define SPECTRAL_METER_TRACKER_H_

#include <ros/ros.h>
//#include <dynamic_reconfigure/server.h>

namespace galt {
namespace spectral_meter {

class Tracker {
 public:
  explicit Tracker(const ros::NodeHandle& pnh);

 private:
  ros::NodeHandle pnh_;
};

}  // namespace spectral_meter
}  // namespace galt

#endif  // SPECTRAL_METER_TRACKER_H_
