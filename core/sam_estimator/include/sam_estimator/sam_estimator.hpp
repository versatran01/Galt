/*
 * sam_estimator.hpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of sam_estimator.
 *
 *	Created on: 21/08/2014
 */

#ifndef GALT_SAM_ESTIMATOR_HPP_
#define GALT_SAM_ESTIMATOR_HPP_

#include <sam_estimator/gtsam.hpp>

namespace galt {
namespace sam_estimator {

class SamEstimator {
public:
  
  SamEstimator();
  
  void addImu();
  
  void addOdometry();
  
  void addGps();
  
  void addLidar();
  
private:
};

}
}

#endif
