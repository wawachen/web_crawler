/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"
#include "lutz/PodCanHeader.h"
#include "lutz/Handshake.h"

namespace tsc_acs {
namespace lutz {

typedef struct
{
    _header header;
    uint8_t podSeed:4;
    uint8_t podKeyStatus:2;
    uint8_t podSessionStatus:2;
    uint32_t podKeyResp;
    uint8_t podCommsQOS;
} __attribute__((packed)) Packet_HANDSHAKE;

const uint64_t stateMask_HANDSHAKE = 0xFFFFFFFFFFFF0000;

void PodRxNode::handlePOD_HANDSHAKE(const can_msgs::Frame & msg)
{
    if(ShouldPublish(msg, stateMask_HANDSHAKE, statePOD_HANDSHAKE))
    {
        Packet_HANDSHAKE & packet = *((Packet_HANDSHAKE*)msg.data.data());

        ::lutz::Handshake msgOut;
        msgOut.header = msg.header;
        ReadPodHeader(msgOut.pod_header, msg);
        msgOut.seed = packet.podSeed;
        msgOut.key_status = packet.podKeyStatus;
        msgOut.session_status = packet.podSessionStatus;
        msgOut.key_response = packet.podKeyResp;
        msgOut.qos = packet.podCommsQOS;

        pubPOD_HANDSHAKE.publish(msgOut);
    }
}

}
}


