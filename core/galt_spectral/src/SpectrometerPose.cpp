/*
 * SensorPoseConfig.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 16/7/2014
 *      Author: gareth
 */

#include "spectral/SpectrometerPose.hpp"
#include "yaml_utilities.hpp"

using namespace galt;

SpectrometerPose::SpectrometerPose()
    : position_(0, 0, 0), direction_(0, 0, 0), fov_(0) {}

SpectrometerPose::SpectrometerPose(const kr::vec3d &position,
                                   const kr::vec3d &direction, double fov)
    : position_(position), direction_(direction), fov_(fov) {}

kr::vec3d SpectrometerPose::getPosition() const { return position_; }

kr::vec3d SpectrometerPose::getDirection() const { return direction_; }

double SpectrometerPose::getFov() const { return fov_; }

YAML::Node YAML::convert<galt::SpectrometerPose>::encode(
    const galt::SpectrometerPose &rhs) {
  YAML::Node node;
  node["position"] = rhs.getPosition();
  node["direction"] = rhs.getDirection();
  node["fov"] = rhs.getFov();
  return node;
}

bool
YAML::convert<galt::SpectrometerPose>::decode(const YAML::Node &node,
                                              galt::SpectrometerPose &rhs) {

  const auto requiredFields = { "position", "direction", "fov" };
  if (!node.IsMap() || !galt::hasFields(node, requiredFields)) {
    return false;
  }

  rhs = galt::SpectrometerPose(node["position"].as<kr::vec3d>(),
                               node["direction"].as<kr::vec3d>(),
                               node["fov"].as<double>());

  return true;
}
