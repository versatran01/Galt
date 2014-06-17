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

#ifndef IMU_H_
#define IMU_H_

#include <memory>
#include <string>
#include <deque>
#include <queue>
#include <vector>

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__   //  will fail outside of gcc/clang
#   define HOST_LITTLE_ENDIAN
#else
#   define HOST_BIG_ENDIAN
#endif

namespace imu_3dm_gx4
{

/**
 *  @brief Imu Interface to the Microstrain 3DM-GX4-25 IMU
 *  @see http://www.microstrain.com/inertial/3dm-gx4-25
 */
class Imu
{
public:
    
    struct Packet
    {
        static constexpr uint8_t kHeaderLength = 4;
        
        static constexpr uint8_t kSyncMSB = 0x75;
        static constexpr uint8_t kSyncLSB = 0x65;
        
        union {
            struct {
                uint8_t syncMSB;
                uint8_t syncLSB;
            };
            uint16_t sync;     /**< Identifer of packet */
        };
        
        uint8_t descriptor;    /**< Type of packet */
        uint8_t length;        /**< Length of the packet in bytes */
        
        uint8_t payload[255];  /**< Payload of packet */
        
        union {
            struct {
                uint8_t checkMSB;
                uint8_t checkLSB;
            };
            uint16_t checksum;  /**< Packet checksum */
        };
    
        /**
         *  @brief is_ack True if this packet corresponds to a [N]ACK message.
         */
        bool is_ack() const;
        
        /**
         *  @brief is_imu_data True if this packet corresponds to an imu data message.
         */
        bool is_imu_data() const;
        
        /**
         * @brief is_filter_data True if this packet corresponds to a filter data message
         */
        bool is_filter_data() const;
        
        /**
         *  @brief ack_code Extract the ACK code from this packet.
         *  @param commmand Command packet to which this ACK should correspond.
         *
         *  @return -1 if the packets do not correspond or this is not an ACK, the error code otherwise.
         */
        int ack_code(const Packet& command) const;
        
        /**
         *  @brief Calculate the packet checksum. Sets the checksum variable
         */
        void calcChecksum();
        
        /**
         *  @brief Constructor
         *  @param desc Major descriptor of this command.
         *  @param len Length of the packet payload.
         */
        Packet(uint8_t desc=0, uint8_t len=0);
        
    } __attribute__((packed));
    
    /**
     *  @brief Info Structure generated by the getDeviceInfo command
     */
    struct Info
    {
        uint16_t firmwareVersion;   /**< Firmware version */
        std::string modelName;      /**< Model name */
        std::string modelNumber;    /**< Model number */
        std::string serialNumber;   /**< Serial number */
        std::string lotNumber;      /**< Lot number - appears to be unused */
        std::string deviceOptions;  /**< Device options (range of the sensor) */
        
        Info() {}
        
        //  non-copyable
        Info(const Info&) = delete;
        Info& operator = (const Info&) = delete;
    };
    
    /**
     *  @brief IMUData IMU readings produced by the sensor
     */
    struct IMUData
    {
        enum
        {
            Accelerometer   = (1 << 0),
            Gyroscope       = (1 << 1),
            Magnetometer    = (1 << 2),
            Barometer       = (1 << 3),
        };
        
        unsigned int fields;    /**< Which fields are valid in the struct */
        
        float accel[3];         /**< Acceleration, units of G */
        float gyro[3];          /**< Angular rates, units of rad/s */
        float mag[3];           /**< Magnetic field, units of gauss */
        float pressure;         /**< Pressure, units of gauss */
        
        IMUData() : fields(0) {}
    };
    
    /**
     *  @brief FilterData Estimator readings produced by the sensor
     */
    struct FilterData
    {
      enum
      {
        Quaternion    = (1 << 0),
      };
      
      unsigned int fields;    /**< Which fields are valid in the struct */
      
      float quaternion[4];    /**< Orientation quaternion (q0,q1,q2,q3) */
      uint16_t quatStatus;    /**< Quaternion status: 0 = invalid, 1 = valid, 2 = georeferenced to magnetic north */
            
      FilterData() : fields(0) {}
    };
    
    /* Exceptions */
    
    /**
     *  @brief io_error Generated when a low-level IO command fails.
     */
    struct io_error : public std::runtime_error {
        io_error(const std::string& desc) : std::runtime_error(desc) {}
    };
    
    /**
     *  @brief timeout_error Generated when write() times out, usually indicates device hang up.
     */
    struct timeout_error : public std::exception {
        timeout_error(uint8_t desc1, uint8_t desc2) : pDesc(desc1), fDesc(desc2) {}
        
        uint8_t pDesc;  //  packet descriptor
        uint8_t fDesc;  //  field descriptor
    };
    
    /* Methods */
    
    /**
     *  @brief Imu Constructor
     *  @param device Path to the device in /dev, eg. /dev/ttyACM0
     */
    Imu(const std::string& device);
    
    /**
     *  @brief ~Imu Destructor
     *  @note Calls disconnect() automatically.
     */
    virtual ~Imu();
    
    /**
     *  @brief connect Open a file descriptor to the serial device.
     *  @throw runtime_error if already open or path is invalid. 
     *  io_error for termios failures.
     */
    void connect();
    
    /**
     *  @brief runOnce Poll for input and read packets if available.
     *  @note kTimeout controls the poll duration, default is 100ms.
     */
    void runOnce();
    
    /**
     *  @brief disconnect Close the file descriptor, sending the IDLE command first.
     */
    void disconnect();
    
    /**
     *  @brief selectBaudRate Select baud rate.
     *  @param baud The desired baud rate. Supported values are: 9600,19200,115200,230400,460800,921600.
     *
     *  @note This command will attempt to communicate w/ the device using all possible baud rates. Once
     *  the current baud rate is determined, it will switch to 'baud' and send the UART command.
     */
    void selectBaudRate(unsigned int baud);
    
    /**
     *  @brief ping Ping the device.
     *  @param to Timeout in milliseconds.
     *
     *  @return 0 on timeout, negative value if NACK is received, positive on success.
     */
    int ping(unsigned int to);
    
    /**
     *  @brief idle Switch the device to idle mode.
     *  @param to Timeout in milliseconds.
     *
     *  @return 0 on timeout, negative value if NACK is received, positive on success.
     */
    int idle(unsigned int to);
    
    /**
     *  @brief resume Resume the device.
     *  @param to Timeout in milliseconds.
     *
     *  @return 0 on timeout, negative value if NACK is received, positive on success.
     */
    int resume(unsigned int to);
    
    /**
     *  @brief getDeviceInfo Get hardware information about the device.
     *  @param info Struct into which information is placed.
     *
     *  @return 0 on timeout, negative value if NACK is received, positive on success.
     */
    int getDeviceInfo(Imu::Info& info);
    
    /**
     *  @brief setIMUDataRate Set imu data rate for different sources.
     *  @param decimation Denominator in the update rate value: 1000/x
     *  @param sources Sources to apply this rate to. May be a bitwise combination of the values:
     *   Accelerometer, Gyroscope, Magnetometer, Barometer
     *
     *  @throw invalid_argument if an invalid source is requested.
     *  @return 0 on timeout, negative value if NACK is received, positive on success.
     */
    int setIMUDataRate(uint16_t decimation, unsigned int sources);
    
    /**
     *  @brief setFilterDataRate Set estimator data rate for different sources.
     *  @param decimation Denominator in the update rate value: 1000/x
     *  @param sources Sources to apply this rate to. May be a bitwise combination of the values:
     *   Quaternion
     *
     *  @throw invalid_argument if an invalid source is requested.
     *  @return 0 on timeout, negative value if NACK is received, positive on success.
     */
    int setFilterDataRate(uint16_t decimation, unsigned int sources); 
    
    /**
     * @brief enableMeasurements Set which measurements to enable in the filter
     * @param accel If true, acceleration measurements are enabled
     * @param magnetometer If true, magnetometer measurements are enabled
     * @return 0 on timeout, negative value if NACK is received, positive on success.
     */
    int enableMeasurements(bool accel, bool magnetometer);
    
    /**
     *  @brief enableIMUStream Enable/disable streaming of IMU data
     *  @param enabled If true, streaming is enabled.
     *
     *  @return 0 on timeout, negative value if NACK is received, positive on success.
     */
    int enableIMUStream(bool enabled);
    
    /**
     *  @brief enableFilterStream Enable/disable streaming of estimation filter data
     *  @param enabled If true, streaming is enabled.
     *
     *  @return 0 on timeout, negative value if NACK is received, positive on success.
     */
    int enableFilterStream(bool enabled);
    
    /* Public properties */
    
    std::function<void (const Imu::IMUData&)> imuDataCallback;        /**< Called with IMU data is ready */
    std::function<void (const Imu::FilterData&)> filterDataCallback;  /**< Called when filter data is ready */
    
private:
    
    //  non-copyable
    Imu(const Imu&) = delete;
    Imu& operator = (const Imu&) = delete;
    
    int pollInput(unsigned int to);
    
    int handleRead(size_t);
    
    void processPacket();
    
    int writePacket(const Packet& p, unsigned int to);
    
    void sendPacket(const Packet& p, unsigned int to);
    
    int receiveResponse(const Packet& command, unsigned int to);
    
    int sendCommand(const Packet &p, unsigned int to);
    
    bool termiosBaudRate(unsigned int baud);
        
    const std::string device_;
    
    int fd_;
    
    std::vector<uint8_t> buffer_;
    std::deque<uint8_t> queue_;
    size_t srcIndex_, dstIndex_;
    
    enum {
        Idle = 0,
        Reading,
    } state_;
    Packet packet_;
};

}  //  imu_3dm_gx4

#endif // IMU_H_
