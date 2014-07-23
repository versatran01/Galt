/*
 * FilterProfile.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 20/7/2014
 *      Author: gareth
 */

#include <spectral/FilterProfile.hpp>
#include "yaml_utilities.hpp"

namespace galt {

FilterProfile::FilterProfile()
    : name_(), center_(0), fwhm_(0), minPeakTransmission_(0) {}

FilterProfile::FilterProfile(const std::string &name, double center,
                             double fwhm, double minPeakTransmission,
                             const galt::Spectrum &spectrum)
    : name_(name), center_(center), fwhm_(fwhm),
      minPeakTransmission_(minPeakTransmission), spectrum_(spectrum) {}

const std::string &FilterProfile::getName() const { return name_; }

const double &FilterProfile::getCenter() const { return center_; }

const double &FilterProfile::getFwhm() const { return fwhm_; }

const double &FilterProfile::getMinPeakTransmission() const {
  return minPeakTransmission_;
}

const galt::Spectrum &FilterProfile::getSpectrum() const { return spectrum_; }

void FilterProfile::setSpectrum(const galt::Spectrum &spectrum) {
  spectrum_ = spectrum;
}
}

YAML::Node YAML::convert<galt::FilterProfile>::encode(const galt::FilterProfile &rhs) {
  YAML::Node node;
  node["name"] = rhs.getName();
  node["center"] = rhs.getCenter();
  node["fwhm"] = rhs.getFwhm();
  node["min_peak_transmission"] = rhs.getMinPeakTransmission();
  node["spectrum"] = rhs.getSpectrum();
  return node;
}

//  TODO: Add better error checking here
bool YAML::convert<galt::FilterProfile>::decode(const YAML::Node &node, galt::FilterProfile &rhs) {
  
  const auto requiredFields = { "name", "center", "fwhm",
                                "min_peak_transmission", "spectrum" };
  if (!node.IsMap() || !galt::hasFields(node, requiredFields)) {
    return false;
  }
  
  rhs = galt::FilterProfile(node["name"].as<std::string>(),
                            node["center"].as<double>(),
                            node["fwhm"].as<double>(),
                            node["min_peak_transmission"].as<double>(),
                            node["spectrum"].as<galt::Spectrum>());
  return true; 
}

YAML::Emitter &operator<<(YAML::Emitter &out,
                          const galt::FilterProfile &filt) {
  
  out << YAML::Block;
  out << YAML::BeginMap;
  out << YAML::Key << "name" << YAML::Value << filt.getName();
  out << YAML::Key << "center" << YAML::Value << filt.getCenter();
  out << YAML::Key << "fwhm" << YAML::Value << filt.getFwhm();
  out << YAML::Key << "min_peak_transmission" << YAML::Value << filt.getMinPeakTransmission();
  out << YAML::Key << "spectrum" << YAML::Value << filt.getSpectrum();
  out << YAML::EndMap;
  return out; 
}
