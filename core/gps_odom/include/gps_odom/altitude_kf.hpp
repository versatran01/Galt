/*
 * altitude_kf.hpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of gps_odom.
 *
 *	Created on: 31/07/2014
 *		  Author: gareth
 */

#ifndef GPS_ODOM_ALTITUDEKF_HPP
#define GPS_ODOM_ALTITUDEKF_HPP

#include <kr_math/base_types.hpp>

namespace gps_odom {

class AltitudeKF {
public:
  using scalar_t = double;  /**< Type used in all numerical calcuations */
  
  AltitudeKF();
  
  void predict(scalar_t accZ, scalar_t accVariance, scalar_t dt);
  
  void updateHeight(scalar_t z, scalar_t zVariance, 
                    scalar_t velZ, scalar_t velVariance);
  
  void updatePressure(scalar_t pressureKpa, scalar_t variance);
  
private:
  kr::vec2<scalar_t> x_;
  kr::mat2<scalar_t> P_;
};

} //  namespace gps_odom

#endif // GPS_ODOM_ALTITUDEKF_HPP
