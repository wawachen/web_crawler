/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"
#include "lutz/PodCanHeader.h"
#include "lutz/Steer.h"

namespace tsc_acs {
namespace lutz {

typedef struct
{
    _header header;
    uint8_t frontActuatorPosition;
    uint8_t rearActuatorPosition;
    uint8_t EPSsysState:4;
    uint8_t steeringWheelPosition:2;
    uint8_t unused:2;
    uint8_t frontMoveState:4;
    uint8_t rearMoveState:4;
    uint16_t EPSError;
} __attribute__((packed)) PacketPOD_EPS_STATUS;

const uint64_t stateMask_POD_EPS_STATUS = 0xFFFFFF3FFFFF0000;

void PodRxNode::handlePOD_EPS_STATUS(const can_msgs::Frame & msg,
                                     uint64_t & currentState,
                                     ros::Publisher & pub)
{
    if(ShouldPublish(msg, stateMask_POD_EPS_STATUS, currentState))
    {
        PacketPOD_EPS_STATUS & packet = *((PacketPOD_EPS_STATUS*)msg.data.data());

        ::lutz::Steer msgOut;

        msgOut.header = msg.header;
        ReadPodHeader(msgOut.pod_header, msg);
        msgOut.front_act_pos = packet.frontActuatorPosition;
        msgOut.rear_act_pos = packet.rearActuatorPosition;
        msgOut.eps_sys_state = packet.EPSsysState;
        msgOut.steer_wheel_position = packet.steeringWheelPosition;
        msgOut.front_mov_state = packet.frontMoveState;
        msgOut.rear_mov_state = packet.rearMoveState;
        msgOut.eps_error = packet.EPSError;

        pub.publish(msgOut);
    }
}

void PodRxNode::handlePOD_EPS_INBOARD_STATUS(const can_msgs::Frame & msg)
{
  handlePOD_EPS_STATUS(msg, statePOD_EPS_INBOARD_STATUS, pubPOD_EPS_INBOARD_STATUS);
}


void PodRxNode::handlePOD_EPS_OUTBOARD_STATUS(const can_msgs::Frame & msg)
{
  handlePOD_EPS_STATUS(msg, statePOD_EPS_OUTBOARD_STATUS, pubPOD_EPS_OUTBOARD_STATUS);
}

}
}

