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
#include "lutz/SessionControl.h"
#include "lutz/Status.h"
#include "pod/PodDemand.h"
#include "pod/AuxiliaryDemand.h"
#include "lutz/PodStates.h"

namespace tsc_acs {
namespace lutz {

//! ROS Node for building LUTZ messages from topic data

class MessageBuilderNode
{
public:
  //! Constructor
  MessageBuilderNode(int16_t ticksPerRadianPerM, int8_t maxSpeed);

  //! Executes node activity
  void spin();

private:
  void publishMessages();
  void buildMessages();

  void rxSessionControl(const ::lutz::SessionControl & msg);
  void rxPodDemand(const ::pod::PodDemand & msg);
  void rxAuxDemand(const ::pod::AuxiliaryDemand & msg);
  void handlePOD_STATUS(const ::lutz::Status & msg);

private:
    ros::NodeHandle node;

    ros::Rate sendRate;

    ros::Subscriber subSession;
    ros::Subscriber subPodDemand;
    ros::Subscriber subAuxDemand;
    ros::Subscriber subPOD_STATUS;

    ros::Publisher pubACS_AUX_POWER_REQ;
    ros::Publisher pubACS_CTRL_CMD1;
    ros::Publisher pubACS_CTRL_CMD2;

    uint64_t podStateRequest;
    uint8_t podState;
//    uint64_t throttlePot;
    uint8_t maxSpeed;
	int16_t ticksPerRadianPerM;


    ::lutz::AuxPowerRequest auxPowerRequest;
    ::lutz::ControlCommand1 controlCommand1;
    ::lutz::ControlCommand2 controlCommand2;
    ::pod::PodDemand podDemand;
    ::pod::AuxiliaryDemand auxDemand;
};

}
}




