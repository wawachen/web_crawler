/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "lutz/AuxPowerRequest.h"
#include "lutz/ControlCommand1.h"
#include "lutz/ControlCommand2.h"
#include "lutz/Handshake.h"
#include "lutz/SessionControl.h"

namespace tsc_acs {
namespace lutz {

//! ROS Node for transmitting LUTZ Pod CAN data.

//! The PodTxNodeconverts the /ACS/XXX messages and converts them to the can
//! frame and transmits them via the can interface.
//! 1. It subscribes to various pod control topics published by the pod control
//! nodes.
//! 2. It publishes various can messages in the can_msgs::frame format the
//! podcanTx topic.

class PodTxNode
{
public:
  //! Constructor

  //! Subscribes to pod specific transmission to transmission topics and
  //! publishses pod_can_tx
  PodTxNode();

  //! Executes node activity
  void spin();

private:
    void transmitACS_AUX_POWER_REQ(const ::lutz::AuxPowerRequest& msg);
    void transmitACS_CTRL_CMD1(const ::lutz::ControlCommand1& msg);
    void transmitACS_CTRL_CMD2(const ::lutz::ControlCommand2& msg);
    void transmitACS_HANDSHAKE(const ::lutz::Handshake& msg);

    void rxSessionControl(const ::lutz::SessionControl & msg);

private:
    ros::NodeHandle node;

    ros::Subscriber subACS_AUX_POWER_REQ;
    ros::Subscriber subACS_CTRL_CMD1;
    ros::Subscriber subACS_CTRL_CMD2;
    ros::Subscriber subACS_HANDSHAKE;
    ros::Subscriber subDEBUG_EMU_CTRL_REQ;

    ros::Subscriber subSessionControl;

    ros::Publisher pub;
    uint8_t frameCount;
    uint8_t sessionId;
};

}
}




