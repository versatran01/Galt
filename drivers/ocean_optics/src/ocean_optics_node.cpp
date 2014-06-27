/*
 * ocean_optics_node.cpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
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

using namespace std;

ros::NodeHandlePtr nh;
ros::Publisher specPub;

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"ocean_optics");
  nh = ros::NodeHandlePtr( new ros::NodeHandle("~") ); 
  specPub = nh->advertise<ocean_optics::Spectrum>("spectrum", 1);
  
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
  const int maxInt = wrapper.getMaximumIntegrationTime(0);
  const int minInt = wrapper.getMinimumIntegrationTime(0);
  
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
  int integrationTime;
  int boxcarWidth;
  int scansToAverage;
  bool correctElecDark;
  
  nh->param("integration_time", integrationTime, 1000);     //  5ms default
  nh->param("boxcar_width", boxcarWidth, 0);                //  no smoothing
  nh->param("scans_to_average", scansToAverage, 1);         //  no averaging
  nh->param("correct_elec_dark", correctElecDark, static_cast<bool>(true));    //  correct electrical dark
  
  if (integrationTime < minInt || integrationTime > maxInt) {
    ROS_ERROR("Integration time unsupported: %i", integrationTime);
    return -1;
  }
  
  wrapper.setExternalTriggerMode(0,0);
  wrapper.setIntegrationTime(0, integrationTime);
  wrapper.setBoxcarWidth(0, boxcarWidth);
  wrapper.setScansToAverage(0, scansToAverage);
  wrapper.setCorrectForElectricalDark(0, static_cast<int>(correctElecDark));
  
  DoubleArray wavelengths, spectrum;
  
  ros::Rate rate(1000000.0 / integrationTime);  //  convert to hz
  while (ros::ok())
  {
    ocean_optics::Spectrum specMsg;
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
  
	// The number of microseconds for the integration time.
//	int integrationTime = 100000;
//	// The number of CCD elements/pixels provided by the spectrometer
//	int numberOfPixels;
//	// Number of Spectrometers actually attached and talking to us
//	int numberOfSpectrometersAttached;
//	// Pixel values from the CCD
//	DoubleArray spectrumArray;
//	// Wavelengths (in nanomenters) corresponding to each CCD element
//	DoubleArray wavelengthArray;
  
//	// Display the number of spectrometers attached to the user
//	cout << " Number of spectrometers found: " <<
//		numberOfSpectrometersAttached << endl;

//	// If there are no attached spectrometers, return...
//	if(numberOfSpectrometersAttached == 0) {
//		return 0;
//	}

//	// Set the integration time of the first spectrometer to 100 ms
//	wrapper.setIntegrationTime(0, integrationTime);
//	cout << "Integration time of the first spectrometer has been set to " <<
//		integrationTime << endl;
//	cout << "Press <enter> to get a spectrum from this spectrometer..." <<
//		endl;
//	getchar();

//	// If we're here, we know the user pressed a key, so get a spectrum...
//	spectrumArray = wrapper.getSpectrum(0);
//	// Get the wavelengths that correspond to the spectrum...
//	wavelengthArray = wrapper.getWavelengths(0);
//	// Find out how long the pixel array is
//	numberOfPixels = spectrumArray.getLength();
//	// Convert the objects to data types native to C++, and print them to the
//	// user
//	double * wavelengths = wavelengthArray.getDoubleValues();
//	double * spectrum = spectrumArray.getDoubleValues();
//	for(int i = 0; i < numberOfPixels; i++) {
//		cout << "Wavelength: " << setprecision(5) << wavelengths[i]
//			<< "     Spectrum: " << spectrum[i]
//			<< endl;
//	}

//	cout << "Spectrum complete, press <enter> to exit the program..." << endl;
//	getchar();

	wrapper.closeAllSpectrometers();
	return 0;
}
