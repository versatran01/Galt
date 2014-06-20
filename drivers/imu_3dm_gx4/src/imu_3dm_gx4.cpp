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

  std::string device;
  int baudrate;
  int imu_decimation, filter_decimation;
  bool use_filter;

  //  load parameters from launch file
  nh.param<std::string>("device", device, "/dev/ttyACM0");
  nh.param<int>("baudrate", baudrate, 115200);
  nh.param<int>("imu_decimation", imu_decimation, 10);
  nh.param<int>("filter_decimation", filter_decimation, 50);
  nh.param<bool>("use_filter", use_filter, false);

  pubIMU = nh.advertise<sensor_msgs::Imu>("imu", 1);
  pubMag = nh.advertise<sensor_msgs::MagneticField>("magnetic_field", 1);
  pubPressure = nh.advertise<sensor_msgs::FluidPressure>("pressure", 1);

  if (use_filter) {
    pubOrientation = nh.advertise<imu_3dm_gx4::Orientation>("orientation", 1);
  }

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

#define assert_throw(command) if ((command) <= 0) { throw std::runtime_error("Failed last command"); }

    log_w("Idling the device");
    assert_throw(imu.idle(100));

    //  read back data rates
    uint16_t baseRate;
    if (imu.getIMUDataBaseRate(baseRate) > 0) {
      log_w("IMU data base rate: %u Hz", baseRate);
    }
    if (imu.getFilterDataBaseRate(baseRate) > 0) {
      log_w("Filter data base rate: %u Hz", baseRate);
    }
    Imu::DiagnosticFields fields;
    if (imu.getDiagnosticInfo(fields) > 0) {
      log_w("Diagnostic fields:");
      log_w("\tModel number: %u", fields.modelNumber);
      log_w("\tSelector flags: %u", fields.selector);
      log_w("\tStatus flags: %u", fields.statusFlags);
      log_w("\tSystem timer (ms): %u", fields.systemTimer);
      log_w("\tNumber of 1PPS pulses since boot: %u", fields.num1PPSPulses);
      log_w("\tLast 1PPS pulse (ms): %u", fields.last1PPSPulse);
      log_w("\tIMU stream enabled: %u", fields.imuStreamEnabled);
      log_w("\tFilter stream enabled: %u", fields.filterStreamEnabled);
      log_w("\tIMU packets dropped: %u", fields.imuPacketsDropped);
      log_w("\tFilter packets dropped: %u", fields.filterPacketsDropped);
      log_w("\tCOM bytes written: %u", fields.comBytesWritten);
      log_w("\tCOM bytes read: %u", fields.comBytesRead);
      log_w("\tCOM number of write overruns: %u", fields.comNumWriteOverruns);
      log_w("\tCOM number of read overruns: %u", fields.comNumReadOverruns);
      log_w("\tUSB bytes written: %u", fields.usbBytesWritten);
      log_w("\tUSB bytes read: %u", fields.usbBytesRead);
      log_w("\tUSB number of write overruns: %u", fields.usbNumWriteOverruns);
      log_w("\tUSB number of read overruns: %u", fields.usbNumReadOverruns);
      log_w("\tNumber of IMU parse errors: %u", fields.numIMUParseErrors);
      log_w("\tNumber of IMU messages: %u", fields.totalIMUMessages);
      log_w("\tLast IMU message (ms): %u", fields.lastIMUMessage);
    }

    log_w("Selecting IMU decimation rate: %u", imu_decimation);
    assert_throw(imu.setIMUDataRate(imu_decimation, Imu::IMUData::Accelerometer | Imu::IMUData::Gyroscope
                        | Imu::IMUData::Magnetometer | Imu::IMUData::Barometer));

    log_w("Selecting filter decimation rate: %u", filter_decimation);
    assert_throw(imu.setFilterDataRate(filter_decimation, Imu::FilterData::Quaternion));

    log_w("Enabling IMU data stream");
    assert_throw(imu.enableIMUStream(true));

    if (use_filter) {
      log_w("Enabling filter data stream");
      assert_throw(imu.enableFilterStream(true));

      log_w("Enabling filter measurements");
      assert_throw(imu.enableMeasurements(true, false));  // TODO: Make this an option
    } else {
      log_w("Disabling filter data stream");
      assert_throw(imu.enableFilterStream(false));
    }

    log_w("Resuming the device");
    assert_throw(imu.resume(300));

#undef assert_throw

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