#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace rm_serial_driver
{

constexpr uint8_t HEADER_IMU    = 0xA1;
constexpr uint8_t HEADER_STATUS = 0xA2;
constexpr uint8_t HEADER_HP     = 0xA3;
constexpr uint8_t HEADER_NAV_TX = 0xB5;

// 0xA1 — 11 bytes, high-frequency
struct ReceiveImuPacket
{
  uint8_t  header = HEADER_IMU;
  float    pitch;
  float    yaw;
  uint16_t checksum = 0;
} __attribute__((packed));

// 0xA2 — 12 bytes, ~10Hz
struct ReceiveStatusPacket
{
  uint8_t  header = HEADER_STATUS;
  uint8_t  game_progress;              //0x0001 游戏阶段 0-未开始 1-准备阶段 2-裁判系统自检 3-五秒倒计时 4-比赛中 5-结算
  uint16_t stage_remain_time;          //0x0001 当前阶段剩余时间
  uint16_t current_hp;                 //0x0201 机器人当前血量
  uint16_t projectile_allowance_17mm;  //0x0208 17mm 弹丸剩余发射次数
  uint8_t  team_colour;                // 1=red 0=blue
  uint8_t  rfid_base;                  //0x0209 己方基地增益点
  uint16_t checksum = 0;
} __attribute__((packed));

// 0xA3 — 17 bytes, ~2Hz
struct ReceiveHpPacket
{
  uint8_t  header = HEADER_HP;
  uint16_t ally_1_robot_hp;            //己方 1 号英雄机器人血量，未上场或被罚下则为 0
  uint16_t ally_2_robot_hp;            //己方 2 号工程机器人血量
  uint16_t ally_3_robot_hp;            //己方 3 号步兵机器人血量
  uint16_t ally_4_robot_hp;            //己方 4 号步兵机器人血量
  uint16_t ally_7_robot_hp;            //己方 7 号哨兵机器人血量
  uint16_t ally_outpost_hp;            //己方前哨站血量
  uint16_t ally_base_hp;               //己方基地血量
  uint16_t checksum = 0;
} __attribute__((packed));

// 0xB5 — 15 bytes, ROS→电控导航速度指令
struct SendNavPacket
{
  uint8_t  header = HEADER_NAV_TX;
  float    vel_x;                      //底盘 x 方向线速度 (m/s)
  float    vel_y;                      //底盘 y 方向线速度 (m/s)
  float    vel_w;                      //底盘角速度 (rad/s)
  uint16_t checksum = 0;
} __attribute__((packed));

inline size_t packetSizeForHeader(uint8_t header)
{
  switch (header) {
    case HEADER_IMU:    return sizeof(ReceiveImuPacket);
    case HEADER_STATUS: return sizeof(ReceiveStatusPacket);
    case HEADER_HP:     return sizeof(ReceiveHpPacket);
    default:            return 0;
  }
}

template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T packet;
  std::memcpy(reinterpret_cast<uint8_t *>(&packet), data.data(),
              std::min(data.size(), sizeof(T)));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendNavPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendNavPacket));
  std::memcpy(packet.data(), reinterpret_cast<const uint8_t *>(&data), sizeof(SendNavPacket));
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
