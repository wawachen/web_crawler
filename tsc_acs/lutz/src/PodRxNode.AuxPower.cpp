/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"
#include "lutz/PodCanHeader.h"
#include "lutz/AuxPower.h"

namespace tsc_acs {
namespace lutz {

typedef struct
{
    _header header;
    bool output0Status:1;
    bool output1Status:1;
    bool output2Status:1;
    bool output3Status:1;
    bool output4Status:1;
    bool output5Status:1;
    bool output6Status:1;
    bool output7Status:1;
    uint8_t unused;
    uint32_t unused2;
} __attribute__((packed)) PacketPOD_AUX_POWER_STATUS;

const uint64_t stateMask_POD_AUX_POWER = 0x0000000000FF0000;

void PodRxNode::handlePOD_AUX_POWER_STATUS(const can_msgs::Frame& msg)
{
    if(ShouldPublish(msg, stateMask_POD_AUX_POWER, statePOD_AUX_POWER_STATUS))
    {
        PacketPOD_AUX_POWER_STATUS & packet = *((PacketPOD_AUX_POWER_STATUS*)msg.data.data());

        ::lutz::AuxPower msgOut;
        msgOut.header = msg.header;
        ReadPodHeader(msgOut.pod_header, msg);
        msgOut.output0 = packet.output0Status;
        msgOut.output1 = packet.output1Status;
        msgOut.output2 = packet.output2Status;
        msgOut.output3 = packet.output3Status;
        msgOut.output4 = packet.output4Status;
        msgOut.output5 = packet.output5Status;
        msgOut.output6 = packet.output6Status;
        msgOut.output7 = packet.output7Status;

        pubPOD_AUX_POWER_STATUS.publish(msgOut);
    }
}

}
}

