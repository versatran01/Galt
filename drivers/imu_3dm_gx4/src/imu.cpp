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
#include "error_handling.hpp"
#include <chrono>
#include <locale>

// trim from start
static inline std::string ltrim(std::string s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}

extern "C" {
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <termios.h>
}

#define kTimeout    (100)

#define u8(x) static_cast<uint8_t>((x))

using namespace imu_3dm_gx4;
using namespace std;

//  swap order of bytes if on a little endian platform
template <size_t sz>
void to_device_order(uint8_t buffer[sz]) {
#ifdef HOST_LITTLE_ENDIAN
    for (size_t i=0; i < sz/2; i++) {
        std::swap(buffer[i], buffer[sz-i-1]);
    }
#endif
}

template <>
void to_device_order<1>(uint8_t buffer[1]) {}

template <typename T>
size_t encode(uint8_t * buffer, const T& t)
{
    static_assert(std::is_fundamental<T>::value, "Only fundamental types may be encoded");
    
    const size_t szT = sizeof(T);
    union {
        T tc;
        uint8_t bytes[szT];
    };
    tc = t;
    
    to_device_order<szT>(bytes);
    
    //  append
    for (size_t i=0; i < szT; i++) {
        buffer[i] = bytes[i];
    }
    
    return szT;
}

template <typename T, typename ...Ts>
size_t encode(uint8_t * buffer, const T& t, const Ts&... ts) {
    
    const size_t sz = encode(buffer, t);
    
    //  recurse
    return sz + encode(buffer+sizeof(T), ts...);
}

template <ssize_t i, size_t N, typename ...Ts>
struct decoder
{
    //  decode the i'th element of N elements w/ types Ts
    void operator () (uint8_t * buffer, std::tuple<Ts...>& output) {
        
        typedef typename std::tuple_element<i, std::tuple<Ts...>>::type type;
        const size_t szT = sizeof(type);
        
        static_assert(std::is_fundamental<type>::value, "Only fundamental types may be decoded");
        
        union {
            type tc;
            uint8_t bytes[szT];
        };
        for (size_t j=0; j < szT; j++) {
            bytes[j] = buffer[j];
        }
        
        to_device_order<szT>(bytes);
        std::get<i>(output) = tc;
        
        decoder<i+1,N,Ts...> dec;
        dec(buffer+szT, output);
    }
};

template <size_t N, typename ...Ts>
struct decoder<N,N,Ts...>
{
    //  do nothing
    void operator () (uint8_t * buffer, std::tuple<Ts...>& output) {}
};

template <typename ...Ts>
void decode(uint8_t * buffer, std::tuple<Ts...>& output) {
    
    decoder<0,sizeof...(Ts),Ts...> dec;
    dec(buffer,output);
}

bool Imu::Packet::is_ack() const {
    //  length must be 4 for the first field in an ACK
    return (payload[0]==0x04 && payload[1]==0xF1);
}

bool Imu::Packet::is_data() const {
    return descriptor==0x80;
}

int Imu::Packet::ack_code(const Packet& command) const {
    
    if ( !is_ack() ) {
        return -1;
    }
    else if (descriptor != command.descriptor) {
        return -1;   //  does not correspond to this command
    }
    else if (payload[2] != command.payload[1]) {
        return -1;   //  first entry in field must match the sent field desc
    }
    
    //  ack, return error code
    return payload[3];
}

void Imu::Packet::calcChecksum()
{
    uint8_t byte1=0,byte2=0;
    
#define add_byte(x) byte1 += (x); \
                    byte2 += byte1;
    
    add_byte(syncMSB);
    add_byte(syncLSB);
    add_byte(descriptor);
    add_byte(length);
    
    for (size_t i=0; i < length; i++) {
        add_byte(payload[i]);
    }
#undef add_byte
    
    checksum = (static_cast<uint16_t>(byte1) << 8) + static_cast<uint16_t>(byte2);
#ifdef HOST_LITTLE_ENDIAN
    std::swap(checkMSB,checkLSB);   //  convert to device order
#endif
}

Imu::Packet::Packet(uint8_t desc, uint8_t len) : syncMSB(kSyncMSB), syncLSB(kSyncLSB),
    descriptor(desc), length(len), checksum(0)
{
}

Imu::Imu(const std::string& device) : device_(device), baudrate_(115200), fd_(0), state_(Idle)
{
    //  buffer for storing reads
    buffer_.resize(kBufferSize);
}

Imu::~Imu() {
    disconnect();
}

void Imu::connect()
{
    if (fd_ > 0) {
        throw std::runtime_error("Socket is already open");
    }
    
    const char * path = device_.c_str();
    fd_ = ::open(path, O_RDWR | O_NOCTTY | O_NDELAY); //  read/write, no controlling terminal, non blocking access
    if (fd_ < 0) {
        throw std::runtime_error("Failed to open device : " + device_);
    }
    
    struct termios toptions;
    if (tcgetattr(fd_, &toptions) < 0) {
        disconnect();
        throw std::runtime_error("Could not retrieve parameters of device : " + device_);
    }
    
    //  set default baud rate
    cfsetispeed(&toptions, baudrate_);
    cfsetospeed(&toptions, baudrate_);
    
    toptions.c_cflag &= ~PARENB;    //  no parity bit
    toptions.c_cflag &= ~CSTOPB;    //  disable 2 stop bits (only one used instead)
    toptions.c_cflag &= ~CSIZE;     //  disable any previous size settings
    toptions.c_cflag &= ~HUPCL;     //  disable modem disconnect
    toptions.c_cflag |= CS8;        //  bytes are 8 bits long
    
    toptions.c_cflag &= ~CRTSCTS;  //  no hardware RTS/CTS switch
    
    toptions.c_cflag |= CREAD | CLOCAL;             //  enable receiving of data, forbid control lines (CLOCAL)
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY);    //  no software flow control
    
    //  disable the following
    //  ICANON = input is read on a per-line basis
    //  ECHO/ECHOE = echo enabled
    //  ISIG = interrupt signals
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    toptions.c_oflag &= ~OPOST;     //  disable pre-processing of input data
    
    toptions.c_cc[VMIN] = 0;        //  no minimum number of bytes when reading
    toptions.c_cc[VTIME] = 0;       //  no time blocking
    
    //  TCSANOW = make change right away
    if (tcsetattr(fd_, TCSANOW, &toptions) < 0) {
        disconnect();
        throw std::runtime_error("Could not set parameters of device : " + device_);
    }
}

void Imu::disconnect()
{
    if (fd_ > 0) {
        close(fd_);
        fd_ = 0;
    }
}

bool Imu::termiosBaudRate(unsigned int baud)
{
    struct termios toptions;
    if (tcgetattr(fd_, &toptions) < 0) {
        return false;
    }
    
    //  modify only the baud rate
    cfsetispeed(&toptions, baud);
    cfsetospeed(&toptions, baud);
    
    if (tcsetattr(fd_, TCSANOW, &toptions) < 0) {
        return false;
    }
    
    usleep(200000); //  wait for connection to be negotiated
                    //  found this length through experimentation
    return true;
}

void Imu::runOnce()
{
    int sig = pollInput(kTimeout);
    if (sig < 0) {
        //  failure in poll/read, device disconnected
        throw galt::Exception( string("Error while polling for input : ") + strerror(errno) );
    }
}

void Imu::selectBaudRate(unsigned int baud)
{
    //  baud rates supported by the 3DM-GX4-25
    const size_t num_rates = 6;
    unsigned int rates[num_rates] = {9600,19200,115200,230400,460800,921600};
    
    if ( !std::binary_search(rates, rates + num_rates, baud) ) {
        //  invalid baud rate
        throw galt::Exception( string("Baud rate unsupported by device : ") + to_string(baud) );
    }
    
    //  place current baudrate first
    for (size_t i=1; i < num_rates; i++) {
        if (rates[i] == baudrate_) {
            std::swap(rates[0], rates[i]);
            break;
        }
    }
    
    bool foundRate = false;
    
    for (size_t i=0; i < num_rates; i++)
    {
        if ( !termiosBaudRate(rates[i]) ) {
            throw galt::Exception( string("Failed to set baud rate to ") + to_string(rates[i]) + " : " + strerror(errno) );
        }
    
        //  ping the device
        if ( ping() ) {
            foundRate = true;
            break;
        }
    }
    
    if (!foundRate) {
        throw galt::Exception("Failed to reach the device : " + device_);
    }
    
    //  we are on the correct baud rate, now change to the new rate
    Packet comm(0x0C, 0x07);
    dbg_assert( encode(comm.payload, u8(0x07), u8(0x40), u8(0x01), static_cast<uint32_t>(baud)) == 0x07 );
    comm.calcChecksum();
    
    if ( !sendCommand(comm, 100) ) {
        throw galt::Exception("Device rejected baud rate : " + to_string(baud));
    }
    
    //  device has switched baud rate, now we should also
    if ( !termiosBaudRate(baud) ) {
        throw galt::Exception("Failed to set baud rate to " + to_string(baud) + " : " + strerror(errno) );
    }
    
    //  ping
    if (!ping()) {
        log_w("Warning: Device did not respond after changing the baud rate!\n");
    }
}

bool Imu::ping()
{
    Imu::Packet p(0x01, 0x02);
    p.payload[0] = 0x02;
    p.payload[1] = 0x01;
    p.calcChecksum();

    return sendCommand(p, kTimeout);
}

bool Imu::idle()
{
    Imu::Packet p(0x01, 0x02);
    p.payload[0] = 0x02;
    p.payload[1] = 0x02;
    p.calcChecksum();
    
    return sendCommand(p, kTimeout);
}

bool Imu::resume()
{
    Imu::Packet p(0x01, 0x02);
    p.payload[0] = 0x02;
    p.payload[1] = 0x06;
    p.calcChecksum();
    
    return sendCommand(p, kTimeout);
}

bool Imu::getDeviceInfo(Imu::Info& info)
{
    Imu::Packet p(0x01, 0x02);
    p.payload[0] = 0x02;
    p.payload[1] = 0x03;
    p.calcChecksum();
 
    if ( sendCommand(p, kTimeout) ) {
        
        to_device_order<2>(&packet_.payload[6]);    //  swap firmware ver.
        memcpy(&info.firmwareVersion, &packet_.payload[6], 2);
        
#define devinfo2str(buf,ofs)  std::string((char*)(buf) + (ofs), 16)
        
        info.modelName = ltrim( devinfo2str(packet_.payload, 8) );
        info.modelNumber = ltrim( devinfo2str(packet_.payload, 24) );
        info.serialNumber = ltrim( devinfo2str(packet_.payload, 40) );
        info.lotNumber = ltrim( devinfo2str(packet_.payload, 56) );
        info.deviceOptions = ltrim( devinfo2str(packet_.payload, 72) );
        
#undef devinfo2str
        
        return true;
    }
    
    return false;
}

bool Imu::setDataRate(uint16_t decimation, unsigned int sources)
{
    Imu::Packet p(0x0C,0x04);
    p.payload[0] = 0x00;    //  field length
    p.payload[1] = 0x08;
    p.payload[2] = 0x01;    //  function
    p.payload[3] = 0x00;    //  descriptor count
    
    uint8_t descCount = 0;
    
    if (sources & Imu::Data::Accelerometer) {
        descCount++;
        p.length += encode(p.payload + p.length, u8(0x04), decimation);
    }
    
    if (sources & Imu::Data::Gyroscope) {
        descCount++;
        p.length += encode(p.payload + p.length, u8(0x05), decimation);
    }
    
    if (sources & Imu::Data::Magnetometer) {
        descCount++;
        p.length += encode(p.payload + p.length, u8(0x06), decimation);
    }
    
    if (sources & Imu::Data::Barometer) {
        descCount++;
        p.length += encode(p.payload + p.length, u8(0x17), decimation);
    }
    
    p.payload[3] = descCount;
    p.payload[0] = 4 + 3*descCount;
    
    p.calcChecksum();
    return sendCommand(p, kTimeout);
}

bool Imu::enableIMUStream(bool enabled)
{
    Packet p(0x0C, 0x05);
    p.payload[0] = 0x05;
    p.payload[1] = 0x11;
    p.payload[2] = 0x01;    //  apply new settings
    p.payload[3] = 0x01;    //  device: IMU
    p.payload[4] = (enabled == true);
    
    p.calcChecksum();
    return sendCommand(p, kTimeout);
}

int Imu::pollInput(unsigned int to)
{
    //  poll socket for inputs
    struct pollfd p;
    p.fd = fd_;
    p.events = POLLIN;
    
    int rPoll = poll(&p, 1, to); // timeout is in millis
    if (rPoll > 0)
    {
        const ssize_t amt = ::read(fd_, &buffer_[0], buffer_.size());
        if (amt > 0) {
            return handleRead(amt);
        }
    } else if (rPoll == 0) {
        return 0;   //  no socket can be read
    }
    
    //  poll() or read() failed
    return -1;
}

int Imu::handleRead(size_t bytes_transferred)  //  parses packets out of the input buffer
{
    //  read data into queue
    for (size_t i=0; i < bytes_transferred; i++) {
        queue_.push_back(buffer_[i]);
    }
    
    if (state_ == Idle) {
        dstIndex_ = 1;  //  reset counts
        srcIndex_ = 0;
    }
    
    while (srcIndex_ < queue_.size())
    {
        if (state_ == Idle)
        {
            //   waiting for packet
            if (queue_.size() > 1)
            {
                if (queue_[0] == Imu::Packet::kSyncMSB && queue_[1] == Imu::Packet::kSyncLSB)
                {
                    //   found the magic ID
                    packet_.syncMSB = queue_[0];
                    state_ = Reading;
                    
                    //  clear out previous packet content
                    memset(packet_.payload, 0, sizeof(packet_.payload));
                }
            }
            
            queue_.pop_front();
            
            dstIndex_ = 1;
            srcIndex_ = 0;
        }
        else if (state_ == Reading)
        {
            uint8_t byte = queue_[srcIndex_];
            const size_t end = Packet::kHeaderLength + packet_.length;
            
            //  fill out fields of packet structure
            if (dstIndex_ == 1)         packet_.syncLSB = byte;
            else if (dstIndex_ == 2)    packet_.descriptor = byte;
            else if (dstIndex_ == 3)    packet_.length = byte;
            else if (dstIndex_ < end)   packet_.payload[dstIndex_ - Packet::kHeaderLength] = byte;
            else if (dstIndex_ == end)  packet_.checkMSB = byte;
            else if (dstIndex_ == end+1)
            {
                state_ = Idle;  //  finished packet
                
                packet_.checkLSB = byte;
                
                //  check checksum
                uint16_t sum = packet_.checksum;
                
                packet_.calcChecksum();
                
                if (sum != packet_.checksum)
                {
                    //  invalid, go back to waiting for a marker in the stream
                    log_w("Warning: Dropped packet with mismatched checksum\n");
                }
                else {
                    //  packet is valid, remove relevant data from queue
                    for (size_t k=0; k <= srcIndex_; k++) {
                        queue_.pop_front();
                    }
                    
                    //  read a packet
                    processPacket();
                    return 1;
                }
            }
            
            dstIndex_++;
            srcIndex_++;
        }
    }
    
    //  no packet
    return 0;
}

void Imu::processPacket()
{
    Data data;
    std::tuple<float,float,float> v3f;
    
    if ( packet_.is_data() )
    {
        //  process all fields in the packet
        size_t idx=0;
        
        while (idx+1 < sizeof(packet_.payload))
        {
            size_t len = packet_.payload[idx];
            size_t ddesc = packet_.payload[idx+1];
            
            if (!len) {
                break;
            }
            
            if (ddesc >= 0x04 && ddesc <= 0x06) //  accel/gyro/mag
            {
                float * buf=0;
                switch (ddesc) {
                    case 0x04: buf = data.accel; break;
                    case 0x05: buf = data.gyro;  break;
                    case 0x06: buf = data.mag;   break;
                };
                dbg_assert(buf != 0);
                
                decode(&packet_.payload[idx + 2], v3f);
                buf[0] = get<0>(v3f);
                buf[1] = get<1>(v3f);
                buf[2] = get<2>(v3f);
            }
            else if (ddesc == 0x17) //  pressure sensor
            {
                std::tuple<float> v1f;
                decode(&packet_.payload[idx + 2], v1f);
                data.pressure = get<0>(v1f);
            }
            else {
                log_w("Warning: Unsupported data field present in IMU packet\n");
            }
            
            idx += len;
        }
        
    }
}

int Imu::writePacket(const Packet& p, unsigned int to)
{
    using namespace std::chrono;
    
    //  place into buffer
    std::vector<uint8_t> v;
    v.reserve(Packet::kHeaderLength+sizeof(Packet::payload)+2);
    
    v.push_back(p.syncMSB);
    v.push_back(p.syncLSB);
    v.push_back(p.descriptor);
    v.push_back(p.length);
    for (size_t i=0; i < p.length; i++) {
        v.push_back(p.payload[i]);
    }
    v.push_back(p.checkMSB);
    v.push_back(p.checkLSB);
    
    auto tstart = high_resolution_clock::now();
    auto tstop = tstart + milliseconds(to);
    
    size_t written = 0;
    while ( written < v.size() )
    {
        const ssize_t amt = ::write(fd_, &v[written], v.size() - written);
        if (amt > 0) {
            written += amt;
        } else if (amt < 0) {
            return -1;  //  error while writing
        }
        
        if (tstop < high_resolution_clock::now()) {
            return 0;   //  timed out
        }
    }
    
    return 1;   //  wrote w/o issue
}

bool Imu::sendCommand(const Packet &p, unsigned int to)
{
    const int wrote = writePacket(p, to);
    if (wrote < 0) {
        throw galt::Exception( string("Error while writing packet : ") + strerror(errno) );
    }
    else if (wrote ==0) {
        throw galt::Exception("Timed out while writing packet");
    }
    
    //  read back response
    auto tstart = chrono::high_resolution_clock::now();
    auto tend = tstart + chrono::milliseconds(to);
    
    while (chrono::high_resolution_clock::now() <= tend)
    {
        if (pollInput(1) > 0)
        {
            //  check if this is an ack
            int ack = packet_.ack_code(p);
            
            if (ack == 0) {
                return true;
            }
            else if (ack > 0) {
                log_w("Warning: Received NACK code %x for command {%x, %x}",
                      ack, p.descriptor, p.payload[1]);
                return false;
            }
        }
    }
    
    return false;
}

