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
  uint8_t emStopRequest:1;
  uint8_t unused:7;
  uint8_t frontSteerAngleRequest;
  uint8_t rearSteerAngleRequest;
  uint8_t unused2;
  uint16_t unused3;
} __attribute__((packed)) PacketACS_CTRL_CMD2;


void PodTxNode::transmitACS_CTRL_CMD2(const ::lutz::ControlCommand2& msg)
{
  can_msgs::Frame controlCommand2;

  controlCommand2.header.frame_id = "";
  controlCommand2.header.stamp = ros::Time::now();
  controlCommand2.id = ACS_CTRL_CMD2;
  controlCommand2.is_rtr = false;
  controlCommand2.is_extended = false;
  controlCommand2.is_error = false;
  controlCommand2.dlc = sizeof(PacketACS_CTRL_CMD2);
  PacketACS_CTRL_CMD2 * pPacket = (PacketACS_CTRL_CMD2 *)controlCommand2.data.c_array();
  pPacket->header.frameCount = frameCount;
  pPacket->header.sessionId = sessionId;
  pPacket->emStopRequest = msg.emergency_stop;
  pPacket->frontSteerAngleRequest = msg.front_steer_angle;
  pPacket->rearSteerAngleRequest = msg.rear_steer_angle;

  pub.publish(controlCommand2);
}

}
}


