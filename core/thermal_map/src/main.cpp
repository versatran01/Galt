#include "thermal_map/thermal_map_node.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "thermal_map");
  ros::NodeHandle nh("~");

  galt::thermal_map::ThermalMapNode thermal_map_node(nh);
  ros::spin();
}
