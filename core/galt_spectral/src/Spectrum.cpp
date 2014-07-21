/*
 * Spectrum.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 17/7/2014
 *      Author: gareth
 */

#include <stdexcept>
#include <limits>
#include <assert.h>

#include <spectral/Spectrum.hpp>
#include "yaml_utilities.hpp"

//  TEMP:
#include <ros/ros.h>

namespace galt {

Spectrum::Spectrum() : wavelengths_(), intensities_() {}

Spectrum::Spectrum(const std::vector<double> &wavelengths,
                   const std::vector<double> &intensities)
    : wavelengths_(wavelengths), intensities_(intensities) {

  if (wavelengths.size() != intensities.size()) {
    throw std::invalid_argument("Wavelength and intensity sizes must match");
  }

  if (!wavelengths.empty()) {
    for (size_t i = 0; i < wavelengths.size() - 1; i++) {
      if (wavelengths[i] >= wavelengths[i + 1]) {
        throw std::invalid_argument("Wavelengths must be sorted and unique");
      }
    }
  }
}

Spectrum::Spectrum(const std::map<double,double>& data) : wavelengths_(), intensities_() {
  wavelengths_.reserve(data.size());
  intensities_.reserve(data.size());
  for (const std::pair<double,double>& p : data) {
    wavelengths_.push_back(p.first);
    intensities_.push_back(p.second);
  }
}

const std::vector<double> &Spectrum::getWavelengths() const {
  return wavelengths_;
}

const std::vector<double> &Spectrum::getIntensities() const {
  return intensities_;
}

std::size_t Spectrum::size() const { return wavelengths_.size(); }

bool Spectrum::hasWavelengths(const std::vector<double>& wavelengths) const {
  if (size() == wavelengths.size()) {
    if (std::equal(wavelengths_.begin(), wavelengths_.end(),
                   wavelengths.begin())) {
      return true;
    }
  }
  return false;
}

void Spectrum::resample(const std::vector<double> &wavelengths) {
  if (wavelengths.empty()) {
    throw std::invalid_argument("Sample wavelengths cannot be empty");
  }

  //  storage for result
  std::vector<double> wOut;
  std::vector<double> iOut;
  wOut.reserve(wavelengths.size());
  iOut.reserve(wavelengths.size());

  size_t I = 0;
  for (const double lambda : wavelengths) {
    double val = std::numeric_limits<double>::quiet_NaN();

    //  first element greater than lambda
    for (; I < wavelengths_.size(); I++) {
      if (wavelengths_[I] > lambda) {
        break;
      }
    }

    if (I == wavelengths_.size()) {
      //  edge case: equal to last value
      if (wavelengths_.back() == lambda) {
        val = intensities_.back();
      } else {
        //  no values greater than lambda
        val = 0.0;
      }
    } else {
      if (I > 0) {
        //  linearly interpolate
        const double x0 = wavelengths_[I - 1];
        const double x1 = wavelengths_[I];
        const double i0 = intensities_[I - 1];
        const double i1 = intensities_[I];

        val = (i1 - i0) * (lambda - x0) / (x1 - x0) + i0;
      } else { //  (I == 0)
        //  edge case: equal to first value
        if (wavelengths_.front() == lambda) {
          val = intensities_.front();
        } else {
          val = 0.0;
        }
      }
    }

    //  check above logic
    assert(val != std::numeric_limits<double>::quiet_NaN());

    wOut.push_back(lambda);
    iOut.push_back(val);
  }

  std::swap(wavelengths_, wOut);
  std::swap(intensities_, iOut);
}

void Spectrum::multiply(const Spectrum &s) {

  if (hasWavelengths(s.getWavelengths())) {
    for (size_t i = 0; i < size(); i++) {
      intensities_[i] *= s.intensities_[i];
    }
    return;
  }
  
  //  wavelengths are not equal, need to resample
  Spectrum rhs = s;
  rhs.resample(wavelengths_);
  
  for (size_t i=0; i < size(); i++) {
    intensities_[i] *= rhs.intensities_[i];
  }
}

void Spectrum::invert() {
  for (size_t i=0; i < size(); i++) {
    intensities_[i] = 1 / intensities_[i];
  }
}

void Spectrum::scale(double s) {
  for (size_t i=0; i < size(); i++) {
    intensities_[i] *= s;
  }
}

double Spectrum::integrate() const {
  
  if (size() <= 1) {
    return 0.0;
  }
  
  double total=0.0;
  for (size_t k=0; k < size() - 1; k++) {
    const double y0 = intensities_[k];
    const double y1 = intensities_[k+1];
    const double x0 = wavelengths_[k];
    const double x1 = wavelengths_[k+1];
    total += (y1+y0)*0.5*(x1-x0);
  }
  
  return total;
}

}

YAML::Node YAML::convert<galt::Spectrum>::encode(const galt::Spectrum &rhs) {
  YAML::Node node;
  node["wavelengths"] = rhs.getWavelengths();
  node["intensities"] = rhs.getIntensities();
  return node;
}

//  TODO: Add exceptions here for missing fields
bool YAML::convert<galt::Spectrum>::decode(const YAML::Node &node, galt::Spectrum &rhs) {
  
  const auto requiredFields = { "wavelengths", "intensities" };
  if (!node.IsMap() || !galt::hasFields(node, requiredFields)) {
    ROS_WARN("Missing field!");
    return false;
  }
  
  YAML::Node W = node["wavelengths"];
  YAML::Node I = node["intensities"];
  if (!W.IsSequence() || !I.IsSequence()) {
    ROS_WARN("Not a sequence!");
    return false;
  }
  
  std::vector<double> wavelengths;
  std::vector<double> intensities;
  
  for (size_t element=0; element < W.size(); element++) {
    wavelengths.push_back(W[element].as<double>());
  }
  for (size_t element=0; element < I.size(); element++) {
    intensities.push_back(I[element].as<double>());
  }
  
  //  will throw if the data is invalid
  rhs = galt::Spectrum(wavelengths,intensities);
  return true;
}

YAML::Emitter &operator<<(YAML::Emitter &out,
                          const galt::Spectrum &spectrum) {
  
  //  use YAML overloaded vector operator to emit
  out << YAML::Block;
  out << YAML::BeginMap;
  out << YAML::Flow;
  out << YAML::Key << "wavelengths" << YAML::Value << spectrum.getWavelengths();
  out << YAML::Key << "intensities" << YAML::Value << spectrum.getIntensities();
  out << YAML::EndMap; 
  return out;
}
