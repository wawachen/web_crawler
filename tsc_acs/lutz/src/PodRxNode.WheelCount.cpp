/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"
#include "lutz/PodCanHeader.h"
#include "lutz/WheelCount.h"

namespace tsc_acs {
namespace lutz {

typedef struct
{
    _header header;
    uint8_t leftFrontWheelCount;
    uint8_t leftRearWheelCount;
    uint8_t rightFrontWheelCount;
    uint8_t rightRearWheelCount;
    uint8_t podSpeed;
    uint8_t unused;
} __attribute__((packed)) PacketPOD_WHL_CNT;

const uint64_t stateMask_POD_WHL_CNT = 0x00FFFFFFFFFF0000;

void PodRxNode::handlePOD_WHL_CNT(const can_msgs::Frame& msg)
{
    if(ShouldPublish(msg, stateMask_POD_WHL_CNT, statePOD_WHL_CNT))
    {
        PacketPOD_WHL_CNT & packet = *((PacketPOD_WHL_CNT*)msg.data.data());

        ROS_INFO("THIS IS POD_WHL_CNT: %x", msg.id);
        ::lutz::WheelCount msgOut;

        msgOut.header = msg.header;
        ReadPodHeader(msgOut.pod_header, msg);
        msgOut.left_front = packet.leftFrontWheelCount;
        msgOut.left_rear = packet.leftRearWheelCount;
        msgOut.right_front = packet.rightFrontWheelCount;
        msgOut.right_rear = packet.rightRearWheelCount;
        msgOut.pod_speed = packet.podSpeed;

        pubPOD_WHL_CNT.publish(msgOut);
    }
}

}
}


