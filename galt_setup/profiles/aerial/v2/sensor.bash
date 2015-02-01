# This file will set the following environmental variable to be read by launch
# files.
export GALT_PLATFORM_NAME=aerial
export GALT_PLATFORM_VER=v2
#All cameras
export GALT_CAMERA_CALIB_DIR=package://galt_setup/profiles/${GALT_PLATFORM_NAME}/${GALT_PLATFORM_VER}/calib/camera

#Color camera
export GALT_COLOR_SERIAL=29900130
export GALT_COLOR_RATE=10
export GALT_COLOR_FRAME=color

#Stereo camera
export GALT_STEREO_LEFT=25001185
export GALT_STEREO_RIGHT=25001206
export GALT_STEREO_RATE=25
export GALT_STEREO_FRAME=stereo

#Laser
export GALT_USE_LASER=false
export GALT_LASER_IP=192.168.0.10
export GALT_LASER_FRAME=laser

#IMU
export GALT_IMU_SERIAL=usb-Lord_Microstrain_Lord_Inertial_Sensor_0000__6234.38162-if00
export GALT_IMU_RATE=250
export GALT_IMU_FRAME=imu

#GPS
export GALT_GPS_SERIAL=usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00
export GALT_GPS_RATE=10
export GALT_GPS_VERSION=8
export GALT_GPS_ENABLE_PPP=false
export GALT_GPS_ENABLE_GLONASS=false
export GALT_GPS_FRAME=gps

#Multispectral
export GALT_USE_SPECTRAL=true
export GALT_SPECTRAL_RATE=5
export GALT_SPECTRAL_670_SERIAL=30000336
export GALT_SPECTRAL_670_FRAME=spectral_670

export GALT_SPECTRAL_690_SERIAL=30000315
export GALT_SPECTRAL_690_FRAME=spectral_690

export GALT_SPECTRAL_570_SERIAL=30000320
export GALT_SPECTRAL_570_FRAME=spectral_570

export GALT_SPECTRAL_530_SERIAL=30000337
export GALT_SPECTRAL_530_FRAME=spectral_530

export GALT_SPECTRAL_700_SERIAL=30000334
export GALT_SPECTRAL_700_FRAME=spectral_700

export GALT_SPECTRAL_800_SERIAL=30000312
export GALT_SPECTRAL_800_FRAME=spectral_800


