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

namespace galt {

/**
 * @brief The FilterProfile class
 */
class FilterProfile {
public:
 
  /**
   * @brief FilterProfile Construct a filter profile with empty parameters.
   */
  FilterProfile();

  /**
   * @brief FilterProfile
   * @param name Human readable name of the filter.
   * @param center Center wavelength in nanometers.
   * @param fwhm Full-width at half-maximum.
   * @param minPeakTransmission Minimum peak transmission.
   * @param spectrum Instance of galt::Spectrum.
   */
  FilterProfile(const std::string &name, double center, double fwhm,
                double minPeakTransmission,
                const galt::Spectrum &spectrum = galt::Spectrum());

  /**
   * @brief Human-readable name of the filter.
   * @return Name as string.
   */
  const std::string &getName() const;

  /**
   * @brief Center wavelength (nm) of the filter.
   * @return Wavelength in nm.
   */
  const double &getCenter() const;

  /**
   * @brief Full-width at half-maximum (nm) of the filter.
   * @note This is the bandwidth at half-transmission.
   * @return FWHM in nm.
   */
  const double &getFwhm() const;

  /**
   * @brief Get the minimum peak transmission of the filter.
   * @return Min. peak transmission, falls in range [0,1).
   */
  const double &getMinPeakTransmission() const;

  /**
   * @brief Get the spectral profile of the filter, where
   * intensities represent transmission rates for each wavelength.
   * @return Instance of galt::Spectrum.
   */
  const galt::Spectrum &getSpectrum() const;

  /**
   * @brief Set the spectral profile of the filter.
   * @param spectrum Instance of galt::Spectrum.
   * @see getSpectrum
   */
  void setSpectrum(const galt::Spectrum &spectrum);

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
YAML::Emitter &operator<<(YAML::Emitter &out, const galt::FilterProfile &pose);

#endif // GALT_SPECTRAL_FILTERPROFILE_HPP_
