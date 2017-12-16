/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"
#include "lutz/PodCanHeader.h"
#include "lutz/Powertrain.h"

namespace tsc_acs {
namespace lutz {

typedef struct
{
    _header header;
    bool gearSelSwitchFwd:1;
    bool gearSelSwitchRev:1;
    bool gearSelSwitchNeu:1;
    bool forwardSelected:1;
    bool reverseSelected:1;
    bool neutralSelected:1;
    bool unused:2;
    uint8_t derating;
    uint8_t vehicleSpeed;
    uint8_t configSpeedLimit;
    uint8_t throttleCh1;
    uint8_t throttleCh2;
} __attribute__((packed)) PacketPOD_POWERTRAIN_STATUS;

const uint64_t stateMask_POD_POWERTRAIN_STATUS = 0xFFFFFFFFFF3F0000;

void PodRxNode::handlePOD_POWERTRAIN_STATUS(const can_msgs::Frame& msg)
{
    if(ShouldPublish(msg, stateMask_POD_POWERTRAIN_STATUS, statePOD_POWERTRAIN_STATUS))
    {
        PacketPOD_POWERTRAIN_STATUS & packet = *((PacketPOD_POWERTRAIN_STATUS*)msg.data.data());

        ::lutz::Powertrain msgOut;
        msgOut.header = msg.header;
        ReadPodHeader(msgOut.pod_header, msg);
        msgOut.switch_forward = packet.gearSelSwitchFwd;
        msgOut.switch_reverse = packet.gearSelSwitchRev;
        msgOut.switch_neutral = packet.gearSelSwitchNeu;
        msgOut.forward_selected = packet.forwardSelected;
        msgOut.reverse_selected = packet.reverseSelected;
        msgOut.neutral_selected = packet.neutralSelected;
        msgOut.derating = packet.derating;
        msgOut.vehicle_speed = packet.vehicleSpeed;
        msgOut.conf_speed_limit = packet.configSpeedLimit;
        msgOut.throttle_ch1 = packet.throttleCh1;
        msgOut.throttle_ch2 = packet.throttleCh2;

        pubPOD_POWERTRAIN_STATUS.publish(msgOut);
    }
}

}
}

