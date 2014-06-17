#include <ros/ros.h>
#include <ros/node_handle.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>

#include <string>

#include <error_handling.hpp>
#include <imu_3dm_gx4/Orientation.h>
#include "imu.hpp"

using namespace imu_3dm_gx4;
using namespace std;

ros::Publisher pubIMU;
ros::Publisher pubMag;
ros::Publisher pubPressure;
ros::Publisher pubOrientation;

void publish_data(const Imu::IMUData& data)
{
    sensor_msgs::Imu imu;
    sensor_msgs::MagneticField field;
    sensor_msgs::FluidPressure pressure;

    imu.header.stamp = ros::Time::now();  //  same timestamp on all published messages
    field.header.stamp = imu.header.stamp;
    pressure.header.stamp = imu.header.stamp;

    imu.orientation_covariance[0] = -1; //  orientation data is on a separate topic

    imu.linear_acceleration.x = data.accel[0];
    imu.linear_acceleration.y = data.accel[1];
    imu.linear_acceleration.z = data.accel[2];

    imu.angular_velocity.x = data.gyro[0];
    imu.angular_velocity.y = data.gyro[1];
    imu.angular_velocity.z = data.gyro[2];

    memset(&imu.linear_acceleration_covariance[0], 0, sizeof(double)*9);  //  no variance available
    memset(&imu.angular_velocity_covariance[0], 0, sizeof(double)*9);

    field.magnetic_field.x = data.mag[0];
    field.magnetic_field.y = data.mag[1];
    field.magnetic_field.z = data.mag[2];

    memset(&field.magnetic_field_covariance[0], 0, sizeof(double)*9);

    pressure.fluid_pressure = data.pressure;
    pressure.variance = 0;

    //  publish
    pubIMU.publish(imu);
    pubMag.publish(field);
    pubPressure.publish(pressure);
}

void publish_filter(const Imu::FilterData& data)
{
  imu_3dm_gx4::Orientation orientation;
  orientation.header.stamp = ros::Time::now();
  orientation.quaternion.w = data.quaternion[0];
  orientation.quaternion.x = data.quaternion[1];
  orientation.quaternion.y = data.quaternion[2];
  orientation.quaternion.z = data.quaternion[3];
  orientation.filterStatus = data.quatStatus;
  
  pubOrientation.publish(orientation); 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_3dm_gx4");

  ros::NodeHandle nh("~");

  pubIMU = nh.advertise<sensor_msgs::Imu>("imu", 1);
  pubMag = nh.advertise<sensor_msgs::MagneticField>("magnetic_field", 1);
  pubPressure = nh.advertise<sensor_msgs::FluidPressure>("pressure", 1);
  pubOrientation = nh.advertise<imu_3dm_gx4::Orientation>("orientation", 1);

  std::string device;
  int baudrate;
  int decimation;

  //  load parameters from launch file
  nh.param<std::string>("device", device, "/dev/ttyACM0");
  nh.param<int>("baudrate", baudrate, 115200);
  nh.param<int>("decimation", decimation, 10);

  Imu imu(device);

  try
  {
    imu.connect();

    log_w("Selecting baud rate %u", baudrate);
    imu.selectBaudRate(baudrate);

    Imu::Info info;
    if ( imu.getDeviceInfo(info) ) {

        log_w("Retrieved device info:");
        log_w("\tFirmware version: %u", info.firmwareVersion);
        log_w("\tModel name: %s", info.modelName.c_str());
        log_w("\tModel number: %s", info.modelNumber.c_str());
        log_w("\tSerial number: %s", info.serialNumber.c_str());
        log_w("\tDevice options: %s", info.deviceOptions.c_str());
    }

    log_w("1. Idling the device");
    if (imu.idle(100) <= 0) {
        throw std::runtime_error("Idle failed");
    }

    log_w("2. Selecting IMU decimation rate: %u", decimation);
    if (imu.setIMUDataRate(decimation, Imu::IMUData::Accelerometer | Imu::IMUData::Gyroscope
                        | Imu::IMUData::Magnetometer | Imu::IMUData::Barometer) <= 0)
    {
        throw std::runtime_error("Setting IMU data rate failed");
    }
    
    log_w("3. Selecting filter decimation rate: %u", 10);
    if (imu.setFilterDataRate(10, Imu::FilterData::Quaternion) <= 0)
    {
      throw std::runtime_error("Set filter data rate failed");
    }

    log_w("4. Enabling IMU data stream");
    if (imu.enableIMUStream(true) <= 0) {
        throw std::runtime_error("Enabling IMU data stream failed");
    }
    
    log_w("5. Enabling filter data stream");
    if (imu.enableFilterStream(true) <= 0) {
      throw std::runtime_error("Enable filter data stream failed");
    }
    
    log_w("6. Enabling filter measurements");
    if (imu.enableMeasurements(true, false) <= 0) {
      throw std::runtime_error("Enable filter measurements failed");
    }
    
    log_w("7. Resuming the device");
    if (imu.resume(100) <= 0) {
        throw std::runtime_error("Resuming the device failed");
    }

    imu.imuDataCallback = publish_data;
    imu.filterDataCallback = publish_filter;
    while (ros::ok()) {
      imu.runOnce();
      //ros::spinOnce();
    }
    imu.disconnect();
  }
  catch (Imu::io_error& e) {
      log_e("IO error: %s\n", e.what());
  }
  catch (Imu::timeout_error& e) {
      log_e("Command write timed out: 0x%02x, 0x%02x\n", e.fDesc, e.pDesc);
  }
  catch (std::exception& e) {
      log_e("Exception: %s\n", e.what());
  }

  return 0;
}
