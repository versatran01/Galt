/*
 * node.hpp
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of pressure_altimeter.
 *
 *	Created on: 24/08/2014
 */

#ifndef GALT_PRESSURE_ALTIMETER_NODE_HPP_
#define GALT_PRESSURE_ALTIMETER_NODE_HPP_

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

namespace galt {
namespace pressure_altimeter {

/**
 * @brief Converts fluid pressure message into height above sea level.
 */
class Node {
public:
  /**
   * @brief Load parameters from ROS param and subscribe to fluid pressure.
   * @param nh Private node handle.
   */
  Node(const ros::NodeHandle& nh);
private:
  
  /**
   * @brief Receive pressure message and convert to height.
   * @param pressure
   */
  void pressureCallback(const sensor_msgs::FluidPressure& pressure);
  
  ros::NodeHandle nh_;
  ros::Subscriber subPressure_;
  ros::Publisher pubHeight_;
  
  //  Physical constants (refer to Wikipedia)
  double kP0 = 101325;    //  Pressure at sea level (Pa)
  double kL = 0.0065;     //  Temperature lapse rate (K/m)
  double kT0 = 288.15;    //  Sea level standard temperature (K)
  double kG = 9.80665;    //  Gravitational acceleration (m/s^2)
  double kM = 0.0289644;  //  Molar mass of dry air (kg / mol)
  double kR = 8.31447;    //  Universal gas constant J/(mol * K)
    
    const double c = kG*kM/(kR*kL);
    
    //  convert from millibar to pascals
    const double pressurePA = fluidPressure->fluid_pressure * 100;
    
    //  calculate height from barometer
    const double lhs = std::log(pressurePA / kP0) * (1 / c);
    const double h = (1 - std::exp(lhs)) * kT0 / kL;  
};

} //  pressure_altimeter
} //  galt

#endif // GALT_PRESSURE_ALTIMETER_NODE_HPP_
