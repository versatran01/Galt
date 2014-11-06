/*
 * yaml_utilities.hpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 16/7/2014
 *      Author: gareth
 */

#ifndef GALT_SPECTRAL_YAML_UTILITIES_HPP
#define GALT_SPECTRAL_YAML_UTILITIES_HPP

#include <yaml-cpp/yaml.h>

namespace galt {

/**
 * @brief Check if a node contains fields in a list.
 * @param node Node to search for fields.
 * @param keys Initializer list of keys, either integers or strings.
 * @param bool True if the node has all the fields.
 */
template <typename T>
bool hasFields(const YAML::Node& node, const std::initializer_list<T>& keys) {
  for (const std::string& key : keys) {
    if (!node[key]) {
      return false;
    }
  }
  return true;
}
}

#endif  // YAML_UTILITIES_HPP
