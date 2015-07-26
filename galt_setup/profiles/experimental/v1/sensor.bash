# This file will set the following environmental variable to be read by launch files.

export GALT_PLATFORM_NAME=experimental
export GALT_PLATFORM_VER=v1

export GALT_CAMERA_CALIB_DIR=package://galt_setup/profiles/${GALT_PLATFORM_NAME}/${GALT_PLATFORM_VER}/calib/camera_info

#Mono camera
export GALT_MONO_SERIAL=25001185
export GALT_MONO_RATE=30
export GALT_MONO_FRAME=mono

#Color camera
export GALT_COLOR_SERIAL=15122690
export GALT_COLOR_RATE=15
export GALT_COLOR_FRAME=color

#Laser
export GALT_USE_LASER=true
export GALT_LASER_IP=192.168.0.10
export GALT_LASER_FRAME=laser

#IMU
export GALT_IMU_SERIAL=usb-Lord_Microstrain_Lord_Inertial_Sensor_0000__6234.21073-if00
export GALT_IMU_RATE=400
export GALT_IMU_FRAME=imu

#GPS
export GALT_GPS_SERIAL=usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00
export GALT_GPS_RATE=10
export GALT_GPS_VERSION=8
export GALT_GPS_ENABLE_PPP=false
export GALT_GPS_ENABLE_GLONASS=false
export GALT_GPS_FRAME=gps

export GALT_USE_SPECTRAL=false
