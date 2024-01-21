// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>
#include <iostream>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A; 
  uint8_t detect_color : 1;  // 0-red 1-blue 发1
  bool reset_tracker;    // 是否重置追踪器 发0
  bool is_shoot;             // 是否开启自瞄模式 开发 1
  float roll;                // rad 
  float yaw;               // rad       
  float pitch;                 // rad
  uint16_t checksum = 0;     // crc16校验位 https://blog.csdn.net/ydyuse/article/details/105395368
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  bool is_tracking;
  float yaw;
  float pitch;
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  // for(int i = 0; i < static_cast<int>(data.size()); i++)
  //                 {
  //                     //int a = int(data_buffer[i])；
  //                     std::cout << "data[" << i << "]:" << data[i] << std::endl;
  //                     //std::cout << "data_buffer[" << i << "]:" << std::hex <<std::uppercase <<a <<std::endl;
  //                 }
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
