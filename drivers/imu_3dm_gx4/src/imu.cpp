/*
 * imu.hpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 2/6/2014
 *		  Author: gareth
 */

#include "imu.hpp"
#include <deque>

using namespace imu_3dm_gx4;
using namespace std;

void Imu::Packet::calcChecksum()
{
  uint8_t byte1=0,byte2=0;

  byte1 += syncMSB;
  byte2 += byte1;

  byte1 += syncLSB;
  byte2 += byte1;

  byte1 += descriptor;
  byte2 += byte1;

  byte1 += length;
  byte2 += byte1;

  for (uint8_t i=0; i < length; i++) {
    byte1 += payload[i];
    byte2 += byte1;
  }

  checksum = (static_cast<uint16_t>(byte1) << 8) + static_cast<uint16_t>(byte2);
}

void Imu::Packet::copyToBuffer(uint8_t *insert)
{
  int index=0;

  insert[index++] = syncMSB;
  insert[index++] = syncLSB;
  insert[index++] = descriptor;
  insert[index++] = length;
  for (uint8_t i=0; i < length; i++) {
    insert[index++] = payload[i];
  }
  insert[index++] = checkMSB;
  insert[index++] = checkLSB;
}

Imu::Imu(ros::NodeHandle comm, ros::NodeHandle params, const boost::asio::io_service& io_service) :
  comm_(comm), params_(params)
{
  port_ = new boost::asio::serial_port(io_service);
}

bool Imu::initialize()
{
  string port;
  unsigned int baudrate;

  params_.params("port", port, "/dev/ttyACM0");
  params_.params("baudrate", baudate, 115200);

  typedef boost::asio::serial_port_base sb;

  try {
      _port->open(port);
  } catch (std::exception& err) {
      throw invalid_argument("Could not open port, error : " + err.what());
  }

  if (!port_->is_open()) {
    throw runtime_error("Failed to open port");
  }

  port_->set_option(sb::baud_rate(baudrate));
  port_->set_option(sb::flow_control::none);
  port_->set_option(sb::parity::none);
  port_->set_option(sb::stop_bits::one);

  setIdle();
  setImuFormat();
  setFilterFormat();
  saveMessageFormat();
  enableDataStream();
  resume();
}

void Imu::run()
{
  deque<uint8_t> data;
  Packet packet;
  bool reading = false;

  while (comm.ok())
  {
    if ( data.empty() )
    {
      uint8_t buffer[4096];
      const size_t readLength = read(buffer, sizeof(buffer));

      for (size_t i=0; i < readLength; i++) {
        data.push_back(buffer[i]);
      }
    }
    else
    {
      if (reading)
      {
        
      }
      else if (data.size() > 1)
      {
        packet.syncMSB = data[0];
        packet.syncLSB = data[1];

        if (packet.syncMSB == 0x75 && packet.syncLSB == 0x65) {
          reading = true;
        }

        data.pop_front();
        data.pop_front();
      }
    }
  }
}

void Imu::ping()
{
  Packet p;
  p.sync1 = 0x75;
  p.sync2 = 0x65;
  p.descriptor = 0x01;
  p.length = 0x02;
  p.payload[0] = 0x02;
  p.payload[1] = 0x01;

  p.calcChecksum();
  if (!write(p)) {
    throw runtime_error("write() failed on ping");
  }
}

void Imu::setIdle()
{

}

void Imu::setImuFormat()
{

}

void Imu::setFilterFormat()
{

}

void Imu::saveMessageFormat() {

}

void Imu::enableDataStream() {

}

void Imu::resume() {

}

// bool poll();

bool Imu::write(const Packet& packet)
{
  uint8_t buffer[6 + 255];
  packet.copyToBuffer(buffer);

  const size_t totalSize = 6 + packet.length;
  return boost::asio::write(*port_, boost::asio::buffer(buffer,totalSize)) == totalSize;
}

size_t Imu::read(void * buffer, size_t length)
{
  return boost::asio::read(*port_, boost::asio::buffer(buffer, length));
}
