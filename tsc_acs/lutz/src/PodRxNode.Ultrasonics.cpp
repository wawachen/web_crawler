/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"
#include "lutz/PodCanHeader.h"
#include "lutz/Ultrasonics.h"

namespace tsc_acs {
namespace lutz {
typedef struct
{
    _header header;
    uint8_t leftOuterUltrasonic;
    uint8_t leftInnerUltrasonic;
    uint8_t rightInnerUltrasonic;
    uint8_t rightOuterUltrasonic;
    uint16_t unused;
} __attribute__((packed)) PacketPOD_ULTRASONIC_STATUS;

const uint64_t stateMask_POD_ULTRASONIC_STATUS = 0x0000FFFFFFFF0000;

void PodRxNode::handlePOD_ULTRASONIC_STATUS(const can_msgs::Frame & msg,
                                            uint64_t & currentState,
                                            ros::Publisher & pub)
{
    if(ShouldPublish(msg, stateMask_POD_ULTRASONIC_STATUS, currentState))
    {
        PacketPOD_ULTRASONIC_STATUS & packet = *((PacketPOD_ULTRASONIC_STATUS*)msg.data.data());

        ::lutz::Ultrasonics msgOut;

        msgOut.header = msg.header;
        ReadPodHeader(msgOut.pod_header, msg);
        msgOut.left_outer = packet.leftOuterUltrasonic;
        msgOut.left_inner = packet.leftInnerUltrasonic;
        msgOut.right_inner = packet.rightInnerUltrasonic;
        msgOut.right_outer = packet.rightOuterUltrasonic;

        pub.publish(msgOut);
    }
}

void PodRxNode::handlePOD_REAR_ULTRASONIC_STATUS(const can_msgs::Frame & msg)
{
  handlePOD_ULTRASONIC_STATUS(msg, statePOD_REAR_ULTRASONIC_STATUS, pubPOD_REAR_ULTRASONIC_STATUS);
}

void PodRxNode::handlePOD_FRONT_ULTRASONIC_STATUS(const can_msgs::Frame & msg)
{
  handlePOD_ULTRASONIC_STATUS(msg, statePOD_FRONT_ULTRASONIC_STATUS, pubPOD_FRONT_ULTRASONIC_STATUS);
}

}
}


