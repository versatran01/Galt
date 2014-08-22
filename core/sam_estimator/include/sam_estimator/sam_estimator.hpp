/*
 * sam_estimator.hpp
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
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

  void AddImu();

  void AddStereo();

  void AddGps();

  void Initialize();

  void Optimize();

 private:
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimates_;
};

}  // namespace sam_estimator
}  // namespace galt

#endif  // GALT_SAM_ESTIMATOR_HPP_
