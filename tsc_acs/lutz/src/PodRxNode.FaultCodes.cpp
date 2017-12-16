/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"
#include "lutz/PodCanHeader.h"
#include "lutz/FaultCodes.h"

namespace tsc_acs {
namespace lutz {

typedef struct
{
    _header header;
    uint8_t podFaultCodes;
    uint8_t swExceptionCount;
    uint32_t unused;
} __attribute__((packed)) PacketPOD_FAULT_CODES;

const uint64_t stateMask_POD_FAULT_CODES = 0x00000000FFFF0000;

void PodRxNode::handlePOD_FAULT_CODES(const can_msgs::Frame& msg)
{
    if(ShouldPublish(msg, stateMask_POD_FAULT_CODES, statePOD_FAULT_CODES))
    {
        PacketPOD_FAULT_CODES & packet = *((PacketPOD_FAULT_CODES*)msg.data.data());

        ::lutz::FaultCodes msgOut;
        msgOut.header = msg.header;
        ReadPodHeader(msgOut.pod_header, msg);
        msgOut.fault_codes = packet.podFaultCodes;
        msgOut.exception_count = packet.swExceptionCount;

        pubPOD_FAULT_CODES.publish(msgOut);
    }
}

}
}


