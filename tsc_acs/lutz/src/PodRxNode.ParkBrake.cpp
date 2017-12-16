/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"
#include "lutz/PodCanHeader.h"
#include "lutz/ParkBrake.h"

namespace tsc_acs {
namespace lutz {

typedef struct
{
    _header header;
    uint8_t frontPressureRaw;
    uint8_t rearPressureRaw;
    uint8_t frontPressureTarget;
    uint8_t rearPressureTarget;
    bool brakeLightOn:1;
    bool ignOn:1;
    bool actOutRlyOn:1;
    bool actInRlyOn:1;
    bool delayedIgn:1;
    bool unused:1;
    bool parkBrakeOn:1;
    bool unused1:1;
    uint8_t systemState;
} __attribute__((packed)) PacketPOD_EPB_STATUS;

const uint64_t stateMask_POD_EPB_STATUS = 0xFF5FFFFFFFFF0000;

void PodRxNode::handlePOD_EPB_STATUS(const can_msgs::Frame& msg)
{
    if(ShouldPublish(msg, stateMask_POD_EPB_STATUS, statePOD_EPB_STATUS))
    {
        PacketPOD_EPB_STATUS & packet = *((PacketPOD_EPB_STATUS*)msg.data.data());

        ::lutz::ParkBrake msgOut;
        msgOut.header = msg.header;
        ReadPodHeader(msgOut.pod_header, msg);
        msgOut.front_pressure_raw = packet.frontPressureRaw;
        msgOut.rear_pressure_raw = packet.rearPressureRaw;
        msgOut.front_pressure_target = packet.frontPressureTarget;
        msgOut.rear_pressure_target = packet.rearPressureTarget;
        msgOut.brake_light = packet.brakeLightOn;
        msgOut.ignition = packet.ignOn;
        msgOut.act_out_rly = packet.actOutRlyOn;
        msgOut.act_in_rly = packet.actInRlyOn;
        msgOut.delayed_ignition = packet.delayedIgn;
        msgOut.park_brake = packet.parkBrakeOn;
        msgOut.system_state = packet.systemState;

        pubPOD_EPB_STATUS.publish(msgOut);
    }
}

}
}


