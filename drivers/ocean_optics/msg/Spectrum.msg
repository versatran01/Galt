# Spectrometer data from the ocean optics driver

std_msgs/Header header

# Integration time used in capture
int32 integrationTime

# Maximum value for intensities of spectra
int32 maxIntensity

# Board and detector temperatures in degrees celsius (-1 if unsupported)
float64 boardTemp
float64 detectorTemp

# Wavelengths
float64[] wavelengths

# Intensities
float64[] spectrum

# Was the detector saturated
bool isSaturated
