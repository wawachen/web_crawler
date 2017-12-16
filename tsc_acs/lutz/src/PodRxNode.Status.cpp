/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"
#include "lutz/PodCanHeader.h"
#include "lutz/Status.h"

namespace tsc_acs {
namespace lutz {

typedef struct
{
    _header header;
    uint8_t podState;
    uint8_t doorOpen:1;
    uint8_t chargeDoorOpen:1;
    uint8_t frontBumperPressed:1;
    uint8_t rearBumperPressed:1;
    uint8_t driverSeatBelt:2;
    uint8_t passengerSeatBelt:2;
    uint8_t motorTemperature;
    uint8_t inverterTemperature;
    uint16_t vbat12Level;
} __attribute__((packed)) PacketPOD_STATUS;

const uint64_t stateMask_POD_STATUS = 0xFFFFFFFFFFFF0000;

void PodRxNode::handlePOD_STATUS(const can_msgs::Frame& msg)
{
    if(ShouldPublish(msg, stateMask_POD_STATUS, statePOD_STATUS))
    {
        PacketPOD_STATUS & packet = *((PacketPOD_STATUS*)msg.data.data());

        ROS_INFO("THIS IS POD_STATUS: %x", msg.id);
        ::lutz::Status msgOut;
        msgOut.header = msg.header;

        msgOut.header = msg.header;
        ReadPodHeader(msgOut.pod_header, msg);
        msgOut.pod_state = packet.podState;
        msgOut.door_open = packet.doorOpen != 0;
        msgOut.charge_door_open = packet.chargeDoorOpen != 0;
        msgOut.front_bumper = packet.frontBumperPressed != 0;
        msgOut.rear_bumper = packet.rearBumperPressed != 0;
        msgOut.driver_seat = packet.driverSeatBelt;
        msgOut.passenger_seat = packet.passengerSeatBelt;
        msgOut.motor_temp = packet.motorTemperature;
        msgOut.inverter_temp = packet.inverterTemperature;
        msgOut.vbat12_level = packet.vbat12Level;

        pubPOD_STATUS.publish(msgOut);
    }
}

}
}
