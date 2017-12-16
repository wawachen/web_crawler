/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"
#include "lutz/PodCanHeader.h"
#include "lutz/Version.h"

namespace tsc_acs {
namespace lutz {

typedef struct
{
    _header header;
    uint8_t ECUidChar0;
    uint8_t ECUidChar1;
    uint8_t ECUidChar2;
    uint8_t optionIdx;
    uint8_t SWVerHi;
    uint8_t SWVerLo;
} __attribute__((packed)) PacketPOD_SW_VER;

const uint64_t stateMask_POD_SW_VER = 0xFFFFFFFFFFFF0000;

void PodRxNode::handlePOD_SW_VER(const can_msgs::Frame & msg,
                                 uint64_t & currentState,
                                 ros::Publisher & pub)
{
    if(ShouldPublish(msg, stateMask_POD_SW_VER, currentState))
    {
        PacketPOD_SW_VER & packet = *((PacketPOD_SW_VER*)msg.data.data());

        ::lutz::Version msgOut;
        msgOut.header = msg.header;
        ReadPodHeader(msgOut.pod_header, msg);
        msgOut.id0 = packet.ECUidChar0;
        msgOut.id1 = packet.ECUidChar1;
        msgOut.id2 = packet.ECUidChar2;
        msgOut.option = packet.optionIdx;
        msgOut.major_version = packet.SWVerHi;
        msgOut.minor_version = packet.SWVerLo;

        pub.publish(msgOut);
    }
}

void PodRxNode::handlePOD_EPB_SW_VER(const can_msgs::Frame & msg)
{
  handlePOD_SW_VER(msg, statePOD_EPB_SW_VER, pubPOD_EPB_SW_VER);
}

void PodRxNode::handlePOD_EPI_SW_VER(const can_msgs::Frame & msg)
{
  handlePOD_SW_VER(msg, statePOD_EPI_SW_VER, pubPOD_EPI_SW_VER);
}

void PodRxNode::handlePOD_EPO_SW_VER(const can_msgs::Frame & msg)
{
  handlePOD_SW_VER(msg, statePOD_EPO_SW_VER, pubPOD_EPO_SW_VER);
}

void PodRxNode::handlePOD_GWY_SW_VER(const can_msgs::Frame & msg)
{
  handlePOD_SW_VER(msg, statePOD_GWY_SW_VER, pubPOD_GWY_SW_VER);
}

void PodRxNode::handlePOD_VMS_SW_VER(const can_msgs::Frame & msg)
{
  handlePOD_SW_VER(msg, statePOD_VMS_SW_VER, pubPOD_VMS_SW_VER);
}

void PodRxNode::handlePOD_WDG_SW_VER(const can_msgs::Frame & msg)
{
  handlePOD_SW_VER(msg, statePOD_WDG_SW_VER, pubPOD_WDG_SW_VER);
}

void PodRxNode::handlePOD_WHL_SW_VER(const can_msgs::Frame & msg)
{
  handlePOD_SW_VER(msg, statePOD_WHL_SW_VER, pubPOD_WHL_SW_VER);
}

}
}
