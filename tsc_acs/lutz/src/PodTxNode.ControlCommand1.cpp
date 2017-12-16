/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodTxNode.h"
#include "lutz/PodCanId.h"
#include "lutz/PodCanHeader.h"
#include "can_msgs/Frame.h"

namespace tsc_acs {
namespace lutz {

typedef struct
{
  _header header;
  uint8_t throttlePot;
  uint8_t direction:2;
  uint8_t hornRequest:1;
  uint8_t unused:1;
  uint8_t parkBrakeOnRequest:1;
  uint8_t parkBrakeOffRequest:1;
  uint8_t speedModeRequest:1;
  uint8_t torqueModeRequest:1;
  uint8_t maxSpeed;
  uint8_t torqueRequest;
  uint8_t podStateRequest;
  uint8_t torqueLimit;
} __attribute__((packed)) PacketACS_CTRL_CMD1;

void PodTxNode::transmitACS_CTRL_CMD1(const ::lutz::ControlCommand1& msg)
{
  can_msgs::Frame controlCommand1;
  controlCommand1.header.frame_id = "";
  controlCommand1.header.stamp = ros::Time::now();
  controlCommand1.id = ACS_CTRL_CMD1;
  controlCommand1.is_rtr = false;
  controlCommand1.is_extended = false;
  controlCommand1.is_error = false;
  controlCommand1.dlc = sizeof(PacketACS_CTRL_CMD1);
  PacketACS_CTRL_CMD1 * pPacket = (PacketACS_CTRL_CMD1 *)controlCommand1.data.c_array();
  pPacket->header.frameCount = frameCount;
  pPacket->header.sessionId = sessionId;
  pPacket->throttlePot = msg.throttle_pot;
  pPacket->direction = msg.direction;
  pPacket->hornRequest = msg.horn;
  pPacket->parkBrakeOnRequest = msg.park_brake_on;
  pPacket->parkBrakeOffRequest = msg.park_brake_off;
  pPacket->speedModeRequest = msg.speed_mode;
  pPacket->torqueModeRequest = msg.torque_mode;
  pPacket->maxSpeed = msg.max_speed;
  pPacket->torqueRequest = msg.torque;
  pPacket->podStateRequest = msg.pod_state_request;
  pPacket->torqueLimit = msg.torque_limit;

  pub.publish(controlCommand1);
}

}
}
