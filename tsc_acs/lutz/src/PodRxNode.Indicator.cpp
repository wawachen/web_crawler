/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"
#include "lutz/PodCanHeader.h"
#include "lutz/Indicator.h"

namespace tsc_acs {
namespace lutz {

typedef struct
{
  _header header;
  uint8_t left:1;
  uint8_t right:1;
  uint8_t unused:6;
  uint8_t unused2;
  uint32_t unused3;
} __attribute__((packed)) PacketPOD_DI_STATE;

const uint64_t stateMask_POD_DI_STATE = 0x0000000000030000;

void PodRxNode::handlePOD_DI_STATE(const can_msgs::Frame & msg)
{
  if(ShouldPublish(msg, stateMask_POD_DI_STATE, statePOD_DI_STATE))
  {
    PacketPOD_DI_STATE & packet = *((PacketPOD_DI_STATE*)msg.data.data());

    ::lutz::Indicator msgOut;
    msgOut.header = msg.header;
    ReadPodHeader(msgOut.pod_header, msg);
    msgOut.left = packet.left != 0;
    msgOut.right = packet.right != 0;

    pubPOD_DI_STATE.publish(msgOut);
  }
}

}
}
