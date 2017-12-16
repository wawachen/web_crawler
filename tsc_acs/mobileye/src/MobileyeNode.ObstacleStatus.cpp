/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "mobileye/MobileyeNode.h"

namespace tsc_acs {
namespace mobileye {

typedef struct
{
  uint8_t numObstacles;
  uint8_t timestamp;
  uint8_t applicationVersion;
  uint8_t activeVersionNumberSection:2;
  uint8_t leftCloseRangCutIn:1;
  uint8_t rightCloseRangCutIn:1;
  uint8_t go:4;
  uint8_t protocolVersion;
  uint8_t closeCar:1;
  uint8_t failsafe:4;
  uint8_t reserved10:3;
  uint16_t unused;
} __attribute__((packed)) PacketDetails_ObstacleStatus;

void MobileyeNode::handleObstacleStatus(const can_msgs::Frame& msg)
{
  PacketDetails_ObstacleStatus & packet = *((PacketDetails_ObstacleStatus*)msg.data.data());

  ::mobileye::ObstacleStatus msgOut;
  msgOut.num_obstacle = packet.numObstacles;
  msgOut.timestamp = packet.timestamp;
  msgOut.application_version = packet.applicationVersion;
  msgOut.active_version_number_section = packet.activeVersionNumberSection;
  msgOut.left_close_rang_cut_in = packet. leftCloseRangCutIn;
  msgOut.right_close_rang_cut_in = packet.rightCloseRangCutIn;
  msgOut.go = packet.go;
  msgOut.protocol_version = packet.protocolVersion;
  msgOut.close_car = packet.closeCar;
  msgOut.failsafe = packet.failsafe;
  msgOut.reserved10 = packet.reserved10;

  pubObstacleStatus.publish(msgOut);
}


}
}
