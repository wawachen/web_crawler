/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"
#include "lutz/PodCanHeader.h"
#include "lutz/Imu1.h"
#include "lutz/Imu2.h"

namespace tsc_acs {
namespace lutz {

typedef struct
{
    uint8_t rate;
    uint32_t unused;
    uint8_t acc;
    uint16_t unused2;
} __attribute__((packed)) PacketPOD_IMU_STATUS;

const uint64_t stateMask_POD_IMU_STATUS = 0x0000FF00000000FF;

void PodRxNode::handlePOD_IMU1_STATUS(const can_msgs::Frame& msg)
{
    if(ShouldPublish(msg, stateMask_POD_IMU_STATUS, statePOD_IMU1_STATUS))
    {
        PacketPOD_IMU_STATUS & packet = *((PacketPOD_IMU_STATUS*)msg.data.data());

        ::lutz::Imu1 msgOut;

        msgOut.header = msg.header;
        msgOut.yaw_rate = packet.rate;
        msgOut.lon_acceleration = packet.acc;

        pubPOD_IMU1_STATUS.publish(msgOut);
    }
}

void PodRxNode::handlePOD_IMU2_STATUS(const can_msgs::Frame& msg)
{
    if(ShouldPublish(msg, stateMask_POD_IMU_STATUS, statePOD_IMU2_STATUS))
    {
        PacketPOD_IMU_STATUS & packet = *((PacketPOD_IMU_STATUS*)msg.data.data());

        ::lutz::Imu2 msgOut;

        msgOut.header = msg.header;
        msgOut.pitch_rate = packet.rate;
        msgOut.lat_acceleration = packet.acc;

        pubPOD_IMU2_STATUS.publish(msgOut);
    }
}

}
}



