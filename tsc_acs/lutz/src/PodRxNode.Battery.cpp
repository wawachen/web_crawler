/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"
#include "lutz/PodCanHeader.h"
#include "lutz/Battery.h"

namespace tsc_acs {
namespace lutz {

typedef struct
{
    _header header;
    uint8_t estopPressed:1;
    uint8_t ignitionManual:1;
    uint8_t ignitionAuto:1;
    uint8_t driveStatus:1;
    uint8_t evMode:4;
    uint8_t stateOfCharge;
    uint16_t batteryPower;
    uint8_t rfEstopPressed:1;
    uint8_t unused:7;
    uint8_t unused2;
} __attribute__((packed)) PacketPOD_BATTERY_STATUS;

const uint64_t stateMask_POD_BATTERY_STATUS = 0x0001FFFFFFFF0000;

void PodRxNode::handlePOD_BATTERY_STATUS(const can_msgs::Frame & msg)
{
    if(ShouldPublish(msg, stateMask_POD_BATTERY_STATUS, statePOD_BATTERY_STATUS))
    {
        PacketPOD_BATTERY_STATUS & packet = *((PacketPOD_BATTERY_STATUS*)msg.data.data());

        ::lutz::Battery msgOut;
        msgOut.header = msg.header;
        ReadPodHeader(msgOut.pod_header, msg);
        msgOut.estop = packet.estopPressed != 0;
        msgOut.ignition_manual = packet.ignitionManual != 0;
        msgOut.ignition_auto = packet.ignitionAuto != 0;
        msgOut.drive_status = packet.driveStatus != 0;
        msgOut.ev_mode = packet.evMode;
        msgOut.state_of_charge = packet.stateOfCharge;
        msgOut.battery_power = packet.batteryPower;
        msgOut.rf_estop = packet.rfEstopPressed != 0;

        pubPOD_BATTERY_STATUS.publish(msgOut);
    }
}

}
}
