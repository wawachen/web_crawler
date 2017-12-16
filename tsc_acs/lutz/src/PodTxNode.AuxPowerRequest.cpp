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
  uint8_t aux0Request:2;
  uint8_t aux1Request:2;
  uint8_t aux2Request:2;
  uint8_t aux3Request:2;
  uint8_t aux4Request:2;
  uint8_t aux5Request:2;
  uint8_t aux6Request:2;
  uint8_t aux7Request:2;
  uint8_t mimicLightRequest;
  uint8_t lhDiRequest:1;
  uint8_t rhDiRequest:1;
  uint8_t hornRequest:1;
  uint8_t unused:5;
  uint16_t unused2;
} __attribute__((packed)) PacketACS_AUX_POWER_REQ;

void PodTxNode::transmitACS_AUX_POWER_REQ(const ::lutz::AuxPowerRequest& msg)
{
  can_msgs::Frame auxPowerRequest;

  auxPowerRequest.header.frame_id = "";
  auxPowerRequest.header.stamp = ros::Time::now();
  auxPowerRequest.id = ACS_AUX_POWER_REQ;
  auxPowerRequest.is_rtr = false;
  auxPowerRequest.is_extended = false;
  auxPowerRequest.is_error = false;
  auxPowerRequest.dlc = sizeof(PacketACS_AUX_POWER_REQ);

  PacketACS_AUX_POWER_REQ * pPacket = (PacketACS_AUX_POWER_REQ *)auxPowerRequest.data.c_array();
  pPacket->header.frameCount = frameCount;
  pPacket->header.sessionId = sessionId;
  pPacket->aux0Request = msg.aux0;
  pPacket->aux1Request = msg.aux1;
  pPacket->aux2Request = msg.aux2;
  pPacket->aux3Request = msg.aux3;
  pPacket->aux4Request = msg.aux4;
  pPacket->aux5Request = msg.aux5;
  pPacket->aux6Request = msg.aux6;
  pPacket->aux7Request = msg.aux7;
  pPacket->mimicLightRequest = msg.mimic_light;
  pPacket->lhDiRequest = msg.left_indicator;
  pPacket->rhDiRequest = msg.right_indicator;
  pPacket->hornRequest = msg.horn;

  pub.publish(auxPowerRequest);
}

}
}


