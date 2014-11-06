/*
 * SpectrometerCalibration.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 22/7/2014
 *      Author: gareth
 */

#include <spectral/SpectrometerCalibration.hpp>
#include "yaml_utilities.hpp"

namespace galt {}

YAML::Node YAML::convert<galt::SpectrometerCalibration>::encode(
    const galt::SpectrometerCalibration &rhs) {
  YAML::Node node;
  node["calibration_date"] = rhs.calibrationDate;
  node["source_spectrum"] = rhs.sourceSpectrum;

  //  encode all samples
  for (size_t i = 0; i < rhs.sampleSpectra.size(); i++) {
    YAML::Node sampleNode = node["sample_spectra"][i];
    sampleNode["reflectance"] = rhs.sampleSpectra[i].reflectance;
    sampleNode["measurement"] = rhs.sampleSpectra[i].measurement;
  }
  return node;
}

bool YAML::convert<galt::SpectrometerCalibration>::decode(
    const Node &node, galt::SpectrometerCalibration &rhs) {

  const auto requiredFields = {"calibration_date", "source_spectrum",
                               "sample_spectra"};
  if (!node.IsMap() || !galt::hasFields(node, requiredFields)) {
    return false;
  }

  rhs.calibrationDate = node["calibration_date"].as<std::string>();
  rhs.sourceSpectrum = node["source_spectrum"].as<galt::Spectrum>();

  YAML::Node samples = node["sample_spectra"];
  if (!samples.IsSequence()) {
    return false;
  }

  for (size_t i = 0; i < samples.size(); i++) {
    galt::SpectrometerCalibration::Sample sample;
    sample.reflectance = samples[i]["reflectance"].as<double>();
    sample.measurement = samples[i]["measurement"].as<galt::Spectrum>();
    rhs.sampleSpectra.push_back(sample);
  }
  return true;
}

YAML::Emitter &operator<<(YAML::Emitter &out,
                          const galt::SpectrometerCalibration &calib) {

  out << YAML::Block;
  out << YAML::BeginMap;
  out << YAML::Key << "calibration_date" << YAML::Value
      << calib.calibrationDate;
  out << YAML::Key << "source_spectrum" << YAML::Value << calib.sourceSpectrum;
  out << YAML::Key << "sample_spectra" << YAML::Value;
  out << YAML::BeginSeq;
  for (size_t i = 0; i < calib.sampleSpectra.size(); i++) {
    out << YAML::BeginMap;
    out << YAML::Key << "reflectance" << YAML::Value
        << calib.sampleSpectra[i].reflectance;
    out << YAML::Key << "measurement" << YAML::Value
        << calib.sampleSpectra[i].measurement;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
  return out;
}
