/*
 * FilterProfile.hpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 20/7/2014
 *      Author: gareth
 */

#ifndef GALT_SPECTRAL_FILTERPROFILE_HPP_
#define GALT_SPECTRAL_FILTERPROFILE_HPP_

#include <spectral/Spectrum.hpp>
#include <yaml-cpp/yaml.h>

//  TODO: Document this
namespace galt {

/**
 * @brief The FilterProfile class
 */
class FilterProfile {
public:
  
  FilterProfile();
  
  FilterProfile(const std::string& name, double center, double fwhm, 
                double minPeakTransmission, 
                const galt::Spectrum& spectrum = galt::Spectrum());
  
  const std::string& getName() const;
  
  const double& getCenter() const;
  
  const double& getFwhm() const;
  
  const double& getMinPeakTransmission() const;
  
  const galt::Spectrum& getSpectrum() const;
  
  void setSpectrum(const galt::Spectrum& spectrum);
  
private:
  std::string name_;
  double center_;
  double fwhm_;
  double minPeakTransmission_;
  galt::Spectrum spectrum_;
};

}

namespace YAML {

/**
 * @brief Encodes FilterProfile to and from yaml.
 */
template <> struct convert<galt::FilterProfile> {
  static Node encode(const galt::FilterProfile &rhs);
  static bool decode(const Node &node, galt::FilterProfile &rhs);
};
}

/**
 * @brief Emit YAML for a filter profile.
 * @param out Emitter to encode to.
 * @param pose Filter profile to encode.
 * @return Emitter.
 */
YAML::Emitter &operator<<(YAML::Emitter &out,
                          const galt::FilterProfile &pose);

#endif // GALT_SPECTRAL_FILTERPROFILE_HPP_
