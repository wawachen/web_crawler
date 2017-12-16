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
    uint8_t ACSSeed:4;
    uint8_t ACSSessionRestartRequest:2;
    uint8_t unused:2;
    uint32_t ACSKeyResponse;
    uint8_t ACSCommsQOS:4;
} __attribute__((packed)) PacketACS_HANDSHAKE;


void PodTxNode::transmitACS_HANDSHAKE(const ::lutz::Handshake& msg)
{
    can_msgs::Frame acsHandshake;

    acsHandshake.header.frame_id = "";
    acsHandshake.header.stamp = ros::Time::now();
    acsHandshake.id = ACS_HANDSHAKE;
    acsHandshake.is_rtr = false;
    acsHandshake.is_extended = false;
    acsHandshake.is_error = false;
    acsHandshake.dlc = sizeof(PacketACS_HANDSHAKE);
    PacketACS_HANDSHAKE * pPacket = (PacketACS_HANDSHAKE *)acsHandshake.data.c_array();
    pPacket->header.frameCount = frameCount++;
    pPacket->header.sessionId = sessionId;
    pPacket->ACSSeed = msg.seed;
    pPacket->ACSSessionRestartRequest = msg.key_status; // TODO: Consider this
    pPacket->ACSKeyResponse = msg.key_response;
    pPacket->ACSCommsQOS = msg.qos;

    pub.publish(acsHandshake);
}

}
}
