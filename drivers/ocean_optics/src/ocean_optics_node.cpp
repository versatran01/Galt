/*
 * ocean_optics_node.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 25/6/2014
 *      Author: gareth
 */

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_msgs/Header.h>

//  Ocean optics components
#include <Wrapper.h>
#include <ArrayTypes.h>

//  custom messages
#include <ocean_optics/Spectrum.h>

// dynamic_reconfigure
#include <dynamic_reconfigure/server.h>
#include <ocean_optics/OceanOpticsConfig.h>

using namespace std;

ros::NodeHandlePtr nh;
ros::Publisher specPub;

std::shared_ptr<dynamic_reconfigure::Server<ocean_optics::OceanOpticsConfig>> drServer;

uint32_t seq=0;

//  application parameters
int integrationTime;
int boxcarWidth;
int scansToAverage;
bool correctElecDark;

//  maximum integration times
int maxInt, minInt;

bool shouldUpdateOptions = false;

//  user changed parameters
void dynamicParamsCallback(ocean_optics::OceanOpticsConfig& config,
                           int32_t level) {
  integrationTime = config.integration_time;
  if (integrationTime > maxInt) {
    integrationTime = maxInt;
  } else if (integrationTime < minInt) {
    integrationTime = minInt;
  }

  boxcarWidth = config.boxcar_width;
  scansToAverage = config.scans_to_average;
  correctElecDark = config.correct_elec_dark;
  shouldUpdateOptions = true;
}

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"ocean_optics");
  nh = ros::NodeHandlePtr( new ros::NodeHandle("~") );
  specPub = nh->advertise<ocean_optics::Spectrum>("spectrum", 1);

  //  reconfigure server
  drServer = std::make_shared<dynamic_reconfigure::Server<ocean_optics::OceanOpticsConfig>>();
  dynamic_reconfigure::Server<ocean_optics::OceanOpticsConfig>::CallbackType cb;
  cb = boost::bind(dynamicParamsCallback, _1, _2);
  drServer->setCallback(cb);

  //  initialize OmniDriver wrapper
  Wrapper wrapper;

  JString apiVer = wrapper.getApiVersion();
  int buildNum = wrapper.getBuildNumber();

  ROS_INFO("OmniDriver version %s [build %i]", apiVer.getASCII(), buildNum);

  const int numSpecs = wrapper.openAllSpectrometers();
  if (numSpecs <= 0) {
    ROS_ERROR("No spectrometers found");
    return -1;
  }
  ROS_INFO("Spectrometers connected: %i", numSpecs);

  //  for now we only handle one spectrometer
  JString name = wrapper.getName(0);
  JString firmwareVersion = wrapper.getFirmwareVersion(0);
  JString serialNumber = wrapper.getSerialNumber(0);
  ROS_INFO("Using spectrometer: %s", name.getASCII());
  ROS_INFO("\tFirmware version: %s", firmwareVersion.getASCII());
  ROS_INFO("\tSerial number: %s", serialNumber.getASCII());

  Bench bench = wrapper.getBench(0);
  if ( bench.isDefined() ) {
    JString filterWavelength = bench.getFilterWavelength();
    JString grating = bench.getGrating();
    JString slitSize = bench.getSlitSize();

    ROS_INFO("\tFilter wavelength: %s", filterWavelength.getASCII());
    ROS_INFO("\tGrating: %s", grating.getASCII());
    ROS_INFO("\tSlit size: %s", slitSize.getASCII());
  }

  //  get the saturation point
  const int numPixels = wrapper.getNumberOfPixels(0);
  const int satPoint = wrapper.getMaximumIntensity(0);
  maxInt = wrapper.getMaximumIntegrationTime(0);
  minInt = wrapper.getMinimumIntegrationTime(0);

  ROS_INFO("\tNumber of pixels: %i", numPixels);
  ROS_INFO("\tSaturation point: %i", satPoint);
  ROS_INFO("\tMaximum integration time: %i us", maxInt);
  ROS_INFO("\tMinimum integration time: %i us", minInt);

  BoardTemperature bTemp;
  if ( wrapper.isFeatureSupportedBoardTemperature(0) ) {
    bTemp = wrapper.getFeatureControllerBoardTemperature(0);
    ROS_INFO("\tBoard temperature: %.3f deg celsius", bTemp.getBoardTemperatureCelsius());
  }

  //  check parameters
  nh->param("integration_time", integrationTime, 1000);     //  5ms default
  nh->param("boxcar_width", boxcarWidth, 0);                //  no smoothing
  nh->param("scans_to_average", scansToAverage, 1);         //  no averaging
  nh->param("correct_elec_dark", correctElecDark, static_cast<bool>(true));    //  correct electrical dark
  shouldUpdateOptions = true;

  if (integrationTime < minInt || integrationTime > maxInt) {
    ROS_ERROR("Integration time unsupported: %i", integrationTime);
    return -1;
  }

  wrapper.setExternalTriggerMode(0,0);

  DoubleArray wavelengths, spectrum;

  ros::Rate rate(1000000.0 / integrationTime);  //  convert to hz
  while (ros::ok())
  {
    if (shouldUpdateOptions) {
      wrapper.setIntegrationTime(0, integrationTime);
      wrapper.setBoxcarWidth(0, boxcarWidth);
      wrapper.setScansToAverage(0, scansToAverage);
      wrapper.setCorrectForElectricalDark(0, static_cast<int>(correctElecDark));
      shouldUpdateOptions = false;
    }

    ocean_optics::Spectrum specMsg;
    specMsg.header.stamp = ros::Time::now();
    specMsg.header.seq = seq++;
    specMsg.boardTemp = -1.0;
    specMsg.detectorTemp = -1.0;
    specMsg.integrationTime = integrationTime;
    specMsg.maxIntensity = satPoint;

    wavelengths = wrapper.getWavelengths(0);
    spectrum = wrapper.getSpectrum(0);

    if (wrapper.isSpectrumValid(0)) {
      specMsg.isSaturated = wrapper.isSaturated(0);

      //  copy data
      specMsg.wavelengths.resize(numPixels);
      specMsg.spectrum.resize(numPixels);

      memcpy(&specMsg.wavelengths[0], wavelengths.getDoubleValues(), sizeof(double)*numPixels);
      memcpy(&specMsg.spectrum[0], spectrum.getDoubleValues(), sizeof(double)*numPixels);

      //  publish
      specPub.publish(specMsg);
    }

    rate.sleep();
    ros::spinOnce();
  }

	wrapper.closeAllSpectrometers();
	return 0;
}
