#include <ros/ros.h>
#include <ros/node_handle.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <string>

#include <error_handling.hpp>
#include <imu_3dm_gx4/FilterStatus.h>
#include "imu.hpp"

using namespace imu_3dm_gx4;
using namespace std;

ros::Publisher pubIMU;
ros::Publisher pubMag;
ros::Publisher pubPressure;
ros::Publisher pubOrientation;
ros::Publisher pubBias;
ros::Publisher pubStatus;

boost::shared_ptr<tf::TransformBroadcaster> tfBroadcaster;
bool broadcast_frame = false;

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
  geometry_msgs::QuaternionStamped orientation;
  orientation.header.stamp = ros::Time::now();
  orientation.quaternion.w = data.quaternion[0];
  orientation.quaternion.x = data.quaternion[1];
  orientation.quaternion.y = data.quaternion[2];
  orientation.quaternion.z = data.quaternion[3];

  pubOrientation.publish(orientation);

  geometry_msgs::Vector3Stamped bias;
  bias.header.stamp = orientation.header.stamp;
  bias.vector.x = data.bias[0];
  bias.vector.y = data.bias[1];
  bias.vector.z = data.bias[2];

  pubBias.publish(bias);

  imu_3dm_gx4::FilterStatus status;
  status.quatStatus = data.quatStatus;
  status.biasStatus = data.biasStatus;
  
  pubStatus.publish(status);
  
  if (broadcast_frame) {
    tf::Transform transform;
    transform.setRotation( tf::Quaternion(data.quaternion[1], data.quaternion[2], data.quaternion[3], data.quaternion[0]) );
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    
    tfBroadcaster->sendTransform( tf::StampedTransform(transform, ros::Time::now(), 
                                                       "fixedFrame", ros::this_node::getName() + "/bodyFrame") );
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_3dm_gx4");

  ros::NodeHandle nh("~");

  std::string device;
  int baudrate;
  int imu_decimation, filter_decimation;
  bool enable_filter;
  bool enable_mag_update;

  //  load parameters from launch file
  nh.param<std::string>("device", device, "/dev/ttyACM0");
  nh.param<int>("baudrate", baudrate, 115200);
  nh.param<int>("imu_decimation", imu_decimation, 10);
  nh.param<int>("filter_decimation", filter_decimation, 5);
  nh.param<bool>("enable_filter", enable_filter, false);
  nh.param<bool>("broadcast_frame", broadcast_frame, false);
  nh.param<bool>("enable_mag_update", enable_mag_update, true);
  
  pubIMU = nh.advertise<sensor_msgs::Imu>("imu", 1);
  pubMag = nh.advertise<sensor_msgs::MagneticField>("magnetic_field", 1);
  pubPressure = nh.advertise<sensor_msgs::FluidPressure>("pressure", 1);

  if (enable_filter) {
    pubOrientation = nh.advertise<geometry_msgs::QuaternionStamped>("orientation", 1);
    pubBias = nh.advertise<geometry_msgs::Vector3Stamped>("bias", 1);
    pubStatus = nh.advertise<imu_3dm_gx4::FilterStatus>("filterStatus", 1);
  }
  
  if (broadcast_frame) {
    tfBroadcaster = boost::shared_ptr<tf::TransformBroadcaster>( new tf::TransformBroadcaster() );
  }

  //  new instance of the IMU
  Imu imu(device);

  try
  {
    imu.connect();

    log_w("Selecting baud rate %u", baudrate);
    imu.selectBaudRate(baudrate);

    Imu::Info info;
    if ( imu.getDeviceInfo(info) ) {
        log_i("Retrieved device info:");
        log_i("\tFirmware version: %u", info.firmwareVersion);
        log_i("\tModel name: %s", info.modelName.c_str());
        log_i("\tModel number: %s", info.modelNumber.c_str());
        log_i("\tSerial number: %s", info.serialNumber.c_str());
        log_i("\tDevice options: %s", info.deviceOptions.c_str());
    }

#define assert_throw(command) if ((command) <= 0) { throw std::runtime_error("Failed last command"); }

    log_i("Idling the device");
    assert_throw(imu.idle(300));

    //  read back data rates
    uint16_t baseRate;
    if (imu.getIMUDataBaseRate(baseRate) > 0) {
      log_i("IMU data base rate: %u Hz", baseRate);
    }
    if (imu.getFilterDataBaseRate(baseRate) > 0) {
      log_i("Filter data base rate: %u Hz", baseRate);
    }
    Imu::DiagnosticFields fields;
    if (imu.getDiagnosticInfo(fields) > 0) {
      log_i("Diagnostic fields:");
      log_i("\tModel number: %u", fields.modelNumber);
      log_i("\tSelector flags: %u", fields.selector);
      log_i("\tStatus flags: %u", fields.statusFlags);
      log_i("\tSystem timer (ms): %u", fields.systemTimer);
      log_i("\tNumber of 1PPS pulses since boot: %u", fields.num1PPSPulses);
      log_i("\tLast 1PPS pulse (ms): %u", fields.last1PPSPulse);
      log_i("\tIMU stream enabled: %u", fields.imuStreamEnabled);
      log_i("\tFilter stream enabled: %u", fields.filterStreamEnabled);
      log_i("\tIMU packets dropped: %u", fields.imuPacketsDropped);
      log_i("\tFilter packets dropped: %u", fields.filterPacketsDropped);
      log_i("\tCOM bytes written: %u", fields.comBytesWritten);
      log_i("\tCOM bytes read: %u", fields.comBytesRead);
      log_i("\tCOM number of write overruns: %u", fields.comNumWriteOverruns);
      log_i("\tCOM number of read overruns: %u", fields.comNumReadOverruns);
      log_i("\tUSB bytes written: %u", fields.usbBytesWritten);
      log_i("\tUSB bytes read: %u", fields.usbBytesRead);
      log_i("\tUSB number of write overruns: %u", fields.usbNumWriteOverruns);
      log_i("\tUSB number of read overruns: %u", fields.usbNumReadOverruns);
      log_i("\tNumber of IMU parse errors: %u", fields.numIMUParseErrors);
      log_i("\tNumber of IMU messages: %u", fields.totalIMUMessages);
      log_i("\tLast IMU message (ms): %u", fields.lastIMUMessage);
    }

    log_i("Selecting IMU decimation rate: %u", imu_decimation);
    assert_throw(imu.setIMUDataRate(imu_decimation, Imu::IMUData::Accelerometer | Imu::IMUData::Gyroscope
                        | Imu::IMUData::Magnetometer | Imu::IMUData::Barometer));

    log_i("Selecting filter decimation rate: %u", filter_decimation);
    assert_throw(imu.setFilterDataRate(filter_decimation, Imu::FilterData::Quaternion | Imu::FilterData::Bias));

    log_i("Enabling IMU data stream");
    assert_throw(imu.enableIMUStream(true));

    if (enable_filter) {
      log_i("Enabling filter data stream");
      assert_throw(imu.enableFilterStream(true));

      log_i("Enabling filter measurements");
      assert_throw(imu.enableMeasurements(true, enable_mag_update));

      log_i("Enabling gyro bias estimation");
      assert_throw(imu.enableBiasEstimation(true));
    } else {
      log_i("Disabling filter data stream");
      assert_throw(imu.enableFilterStream(false));
    }

    log_i("Resuming the device");
    assert_throw(imu.resume(300));

#undef assert_throw

    imu.imuDataCallback = publish_data;
    imu.filterDataCallback = publish_filter;
    
    while (ros::ok()) {
      imu.runOnce();
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
