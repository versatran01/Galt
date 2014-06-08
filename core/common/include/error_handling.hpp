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

namespace galt	//	project galt
{
    
/**
 * @brief Class for handling exceptions w/ detailed error info.
 * @note Use of this class on memory constrained platforms is not recommended.
 * @see http://www.boost.org/community/error_handling.html
 */
class Exception : public std::exception, virtual public boost::exception
{
public:

	/**
	 *	@brief Default ctor
	 */
	Exception() noexcept : description_() {}

	/**
	 *	@brief Ctor
	 *	@param desc Description of failure
	 *	
	 *	@note If desc is null, std::terminate will be triggered.
	 */
	Exception(const char * desc)
	{
		dbg_assert(desc);
		description_ = std::string(desc);
	}

	/**
	 *	@brief Ctor
	 *	@see The const char* version of this function
	 */
	Exception(const std::string& desc) : description_(desc) {}

	/**
	 *	@brief Copy Ctor
	 */
	Exception(const Exception& e) : std::exception(e), boost::exception(e), description_(e.description_) {}

	/**
	 *	@brief operator= Copy operator
	 */
	Exception& operator = (const Exception& e) {
		std::exception::operator = (e);
		boost::exception::operator = (e);
		description_ = e.description_;
		return *this;
	}
  	
  	/**
  	 *	@brief what C-string representation of the error description
  	 */
  	virtual const char* what() const noexcept {
  		return description_.c_str();
  	}

  	/**
  	 *	@brief description Description of the exception triggered
  	 */
  	const std::string& description() const noexcept {
  		return description_;
  	}

private:
	std::string description_;
};
    
template <typename T>
void log_exception(const T&) {
	log_e("Exception: unknown type %s\n", typeid(T).name());
}
    
template <>
void log_exception<std::exception>(const std::exception& e) {
	log_e("Exception: %s\n", e.what());
}
    
//  TODO: Add custom loggers for boost::exception

};	//	namespace galt

#endif //	ERRORHANDLING_H_
