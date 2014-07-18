/*
 * SpectrometerPose.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 16/7/2014
 *      Author: gareth
 */

#include <type_traits>

#include "spectral/SpectrometerPose.hpp"
#include "yaml_utilities.hpp"

namespace galt {

SpectrometerPose::SpectrometerPose()
    : position_(0, 0, 0), direction_(0, 0, 0), fov_(0), squaredError_(0) {}

SpectrometerPose::SpectrometerPose(const kr::vec3d &position,
                                   const kr::vec3d &direction, double fov,
                                   double squaredError)
    : position_(position), direction_(direction), fov_(fov),
      squaredError_(squaredError) {}

const kr::vec3d &SpectrometerPose::getPosition() const { return position_; }

const kr::vec3d &SpectrometerPose::getDirection() const { return direction_; }

const double &SpectrometerPose::getFov() const { return fov_; }

const double &SpectrometerPose::getSquaredError() const { return squaredError_; }

double SpectrometerPose::distanceToPlane(const kr::vec3d &o,
                                         const kr::vec3d &n) const {
  const kr::vec3d del = o - position_;

  const double num = (del[0]*n[0] + del[1]*n[1] + del[2]*n[2]);
  const double den = (direction_[0]*n[0] + direction_[1]*n[1] + direction_[2]*n[2]);

  if (std::abs(den) < std::numeric_limits<double>::epsilon()*10) {
    return std::numeric_limits<double>::infinity();
  }

  return num / den;
}

} // namespace galt

YAML::Node YAML::convert<galt::SpectrometerPose>::encode(
    const galt::SpectrometerPose &rhs) {
  YAML::Node node;
  node["position"] = rhs.getPosition();
  node["direction"] = rhs.getDirection();
  node["fov"] = rhs.getFov();
  node["squared_error"] = rhs.getSquaredError();
  return node;
}

bool
YAML::convert<galt::SpectrometerPose>::decode(const YAML::Node &node,
                                              galt::SpectrometerPose &rhs) {

  const auto requiredFields = { "position", "direction", "fov",
                                "squared_error" };
  if (!node.IsMap() || !galt::hasFields(node, requiredFields)) {
    return false;
  }

  rhs = galt::SpectrometerPose(
      node["position"].as<kr::vec3d>(), node["direction"].as<kr::vec3d>(),
      node["fov"].as<double>(), node["squared_error"].as<double>());

  return true;
}
