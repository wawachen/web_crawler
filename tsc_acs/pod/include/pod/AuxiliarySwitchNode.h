/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "pod/AuxiliaryDemand.h"

namespace tsc_acs {
namespace lutz {

//! ROS Node for turning taking keyboard entry and publishing it.
class AuxiliarySwitchNode
{
public:
	//! Constructor
	AuxiliarySwitchNode();

	//! Executes node activity
	void spin();

private:
	void handlePublisher();
	void handleXnavAuto(const ::pod::AuxiliaryDemand & msg);
	void handleXboxJoy(const ::pod::AuxiliaryDemand & msg);
	void handleMavJoy(const ::pod::AuxiliaryDemand & msg);

private:
	ros::Subscriber subAcsAuxDemand;

	ros::Subscriber subXboxAuxDemand;
	ros::Subscriber subTurnigyAuxDemand;

	ros::NodeHandle node;
	ros::Publisher pub;

	ros::Rate sendRate;

	::pod::AuxiliaryDemand acsAuxDemand;
	::pod::AuxiliaryDemand xboxAuxDemand;
	::pod::AuxiliaryDemand turnigyAuxDemand;
	::pod::AuxiliaryDemand auxiliaryDemand;

};
}}
