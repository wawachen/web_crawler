/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"
#include "lutz/PodCanHeader.h"
#include "lutz/Debug.h"

namespace tsc_acs {
namespace lutz {

typedef struct
{
    uint64_t debugState;
} __attribute__((packed)) PacketPOD_DEBUG;

const uint64_t stateMask_POD_DEBUG = 0xFFFFFFFFFFFFFFFF;

void PodRxNode::handlePOD_DEBUG_COMMON(const can_msgs::Frame & msg,
                                uint64_t & currentState,
                                ros::Publisher & pub)
{
    if(ShouldPublish(msg, stateMask_POD_DEBUG, currentState))
    {
        PacketPOD_DEBUG & packet = *((PacketPOD_DEBUG*)msg.data.data());

        ::lutz::Debug msgOut;
        msgOut.header = msg.header;
        msgOut.debug = packet.debugState;

        pub.publish(msgOut);
    }
}

void PodRxNode::handlePOD_DEBUG(const can_msgs::Frame & msg)
{
  handlePOD_DEBUG_COMMON(msg, statePOD_DEBUG, pubPOD_DEBUG);
}

void PodRxNode::handlePOD_DEBUG2(const can_msgs::Frame & msg)
{
  handlePOD_DEBUG_COMMON(msg, statePOD_DEBUG2, pubPOD_DEBUG2);
}

}
}
