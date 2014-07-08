/*
 * error_handling.hpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 6/6/2014
 *		Author: gareth
 */

#ifndef ERRORHANDLING_H_
#define ERRORHANDLING_H_

#ifdef ROS_PACKAGE_NAME
#	include <ros/ros.h>
#	define BUILD_ROS
#endif

#include <stdexcept>
#include <string>
#include <iostream>
#include <boost/exception/all.hpp>

//  temp
#define BUILD_DEBUG

//	logging macros
#if defined(BUILD_ROS)
#   define log_i(...)   ROS_INFO(__VA_ARGS__)
#   define log_w(...)   ROS_WARN(__VA_ARGS__)
#   define log_e(...)   ROS_ERROR(__VA_ARGS__)
#else
#   define log_i(...)   fprintf(stdout,__VA_ARGS__)
#   define log_w(...)   fprintf(stdout,__VA_ARGS__)
#   define log_e(...)   fprintf(stderr,__VA_ARGS__)
#endif

#if defined(BUILD_DEBUG)
extern "C" {
#	include <assert.h>
}
#	define dbg_assert(a)	assert((a))
#else
#	define dbg_assert(a)
 	//	silence verbose logging
#	undef log_i
#	define log_i(...)
#endif

namespace galt
{
};	//	namespace galt

#endif //	ERRORHANDLING_H_
