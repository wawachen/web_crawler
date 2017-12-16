/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"
#include "lutz/PodCanHeader.h"
#include "lutz/EPS.h"

namespace tsc_acs {
namespace lutz {

typedef struct
{
    _header header;
    uint8_t manualSteerAngle;
    uint8_t inboardEPSState:4;
    uint8_t unused:4;
    uint8_t outboardEPSState:4;
    uint8_t unused2:4;
    uint8_t inboardEPSError;
    uint8_t outboardEPSError;
    uint8_t brakePedalForce;
} __attribute__((packed)) PacketPOD_EPS_STATUS;

const uint64_t stateMask_POD_EPS_STATUS = 0xFFFFFFF0F0FF0000;

void PodRxNode::handlePOD_EPS_STATUS(const can_msgs::Frame& msg)
{
    if(ShouldPublish(msg, stateMask_POD_EPS_STATUS, statePOD_EPS_STATUS))
    {
        PacketPOD_EPS_STATUS & packet = *((PacketPOD_EPS_STATUS*)msg.data.data());

        ::lutz::EPS msgOut;
        msgOut.header = msg.header;
        ReadPodHeader(msgOut.pod_header, msg);
        msgOut.manual_steer_angle = packet.manualSteerAngle;
        msgOut.inboard_eps = packet.inboardEPSState;
        msgOut.outboard_eps = packet.outboardEPSState;
        msgOut.inboard_eps_error = packet.inboardEPSError;
        msgOut.outboard_eps_error = packet.outboardEPSError;
        msgOut.brake_pedal_force = packet.brakePedalForce;

        pubPOD_EPS_STATUS.publish(msgOut);
    }
}

}
}

