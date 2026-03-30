// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{


struct ReceiveNavPacket
{
  uint8_t header=0x5B;
  float pitch;
  float yaw;
  uint8_t game_progress;                //0x0001 游戏阶段 0-未开始 1-准备阶段 2-裁判系统自检 3-五秒倒计时 4-比赛中 5-结算
  uint8_t behavior_state;               //决策，共三档
  uint16_t stage_remain_time;           //0x0001 当前阶段剩余时间
  bool base_gain_point;                 //0x0209 己方基地增益点
  uint16_t current_hp;                  //0x0201 机器人当前血量
                                        //0x0003
  uint16_t red_1_robot_hp;               //红 1 英雄机器人血量。若该机器人未上场或者被罚下，则血量为 0
  uint16_t red_2_robot_hp;               //红 2 工程机器人血量
  uint16_t red_3_robot_hp;               //红 3 步兵机器人血量
  uint16_t red_4_robot_hp;               //红 4 步兵机器人血量
  uint16_t red_7_robot_hp;               //红 7 哨兵机器人血量
  uint16_t red_outpost_hp;               //红方前哨站血量
  uint16_t red_base_hp;                  //红方基地血量
  uint16_t blue_1_robot_hp;              //蓝 1 英雄机器人血
  uint16_t blue_2_robot_hp;              //蓝 2 工程机器人血量
  uint16_t blue_3_robot_hp;              //蓝 3 步兵机器人血量
  uint16_t blue_4_robot_hp;              //蓝 4 步兵机器人血量
  uint16_t blue_7_robot_hp;              //蓝 7 哨兵机器人血量
  uint16_t blue_outpost_hp;              //蓝方前哨站血量
  uint16_t blue_base_hp;                 //蓝方基地血量
  uint16_t team_colour;                  //1 red
  uint16_t projectile_allowance_17mm;   //0x0208 17mm 弹丸剩余发射次数
  float tracker_x;
  float tracker_y;
  bool tracking;
  uint16_t check_sum=0;
  /* data */
}__attribute__((packed));


struct SendNavPacket
{
  uint8_t header=0xB5;
  // float aim_lx; //linear x
  // float aim_ly; //linear y
  // float aim_az; //angular z
  float vel_x;
  float vel_y;
  float vel_v; //vector velocity
  uint16_t checksum=0;
  /* data */
}__attribute__((packed)); //告诉编译器取消结构在编译过程中的优化对齐


inline ReceiveNavPacket fromVector(const std::vector<uint8_t> & data)
{
  ReceiveNavPacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendNavPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendNavPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendNavPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
