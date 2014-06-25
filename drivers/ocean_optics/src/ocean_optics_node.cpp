#include <stdio.h>

#include <Wrapper.h>
#include <ArrayTypes.h>
#include <iostream>
#include <iomanip>

using namespace std;

int main()
{
	// The number of microseconds for the integration time.
	int integrationTime = 100000;
	// The number of CCD elements/pixels provided by the spectrometer
	int numberOfPixels;
	// Number of Spectrometers actually attached and talking to us
	int numberOfSpectrometersAttached;
	// Pixel values from the CCD
	DoubleArray spectrumArray;
	// Wavelengths (in nanomenters) corresponding to each CCD element
	DoubleArray wavelengthArray;
	// THis is the object through wich we will access all of OmniDriver's
	// capabilities
	Wrapper wrapper;

	// Get an array of spectrometer objects...
	numberOfSpectrometersAttached = wrapper.openAllSpectrometers();

	// Display the number of spectrometers attached to the user
	cout << " Number of spectrometers found: " <<
		numberOfSpectrometersAttached << endl;

	// If there are no attached spectrometers, return...
	if(numberOfSpectrometersAttached == 0) {
		return 0;
	}

	// Set the integration time of the first spectrometer to 100 ms
	wrapper.setIntegrationTime(0, integrationTime);
	cout << "Integration time of the first spectrometer has been set to " <<
		integrationTime << endl;
	cout << "Press <enter> to get a spectrum from this spectrometer..." <<
		endl;
	getchar();

	// If we're here, we know the user pressed a key, so get a spectrum...
	spectrumArray = wrapper.getSpectrum(0);
	// Get the wavelengths that correspond to the spectrum...
	wavelengthArray = wrapper.getWavelengths(0);
	// Find out how long the pixel array is
	numberOfPixels = spectrumArray.getLength();
	// Convert the objects to data types native to C++, and print them to the
	// user
	double * wavelengths = wavelengthArray.getDoubleValues();
	double * spectrum = spectrumArray.getDoubleValues();
	for(int i = 0; i < numberOfPixels; i++) {
		cout << "Wavelength: " << setprecision(5) << wavelengths[i]
			<< "     Spectrum: " << spectrum[i]
			<< endl;
	}

	cout << "Spectrum complete, press <enter> to exit the program..." << endl;
	getchar();

	// Close all spectrometers
	wrapper.closeAllSpectrometers();

	return 0;
}
