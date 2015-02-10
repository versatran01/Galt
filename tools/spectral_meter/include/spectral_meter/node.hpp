#ifndef GALT_SPECTRAL_METER_NODE_HPP
#define GALT_SPECTRAL_METER_NODE_HPP

#include <ros/node_handle.h>

namespace galt {
namespace spetral_meter {

class Node {
public:
  
  Node(const ros::NodeHandle& pnh) : pnh_(pnh) {}
  
private:
  ros::NodeHandle pnh_;
};

} //  spectral_meter
} //  galt

#endif  //   GALT_SPECTRAL_METER_NODE_HPP
