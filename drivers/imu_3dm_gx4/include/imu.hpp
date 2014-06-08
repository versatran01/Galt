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
            uint16_t sync;     /**< Header of packet */
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
        
        size_t totalLength() const {
            return kHeaderLength + length + 2;
        }
        
        /**
         *
         */
        bool is_ack() const;
        
        /**
         *
         */
        int ack_code(const Packet& command) const;
        
        /**
         *
         */
        void calcChecksum();
        
        /**
         *  @brief Constructor
         */
        Packet(uint8_t desc=0, uint8_t len=0);
        
    } __attribute__((packed));
    
    struct Info
    {
        uint64_t firmwareVersion;
        std::string modelName;
        std::string modelNumber;
        std::string serialNumber;
        std::string lotNumber;
        std::string deviceOptions;
    };
    
    typedef std::shared_ptr<Imu> Ptr;
    
    /* Constants */
    
    static constexpr size_t kBufferSize = 512;
    
    Imu(const std::string& device);
    
    virtual ~Imu();
    
    void connect();
    
    void runOnce();
    
    void disconnect();
    
    void selectBaudRate(unsigned int baud);
    
    bool ping();
    
    bool idle();
    
    bool resume();
    
    bool getDeviceInfo(Imu::Info& info);
    
public:
        
    int pollInput(unsigned int to);
    
    int handleRead(size_t);
    
    void processPacket();
    
    int writePacket(const Packet& p, unsigned int to);
    
    bool sendCommand(const Packet &p, unsigned int to);
    
    bool termiosBaudRate(unsigned int baud);
        
private:
    const std::string device_;
    unsigned int baudrate_;
    
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
