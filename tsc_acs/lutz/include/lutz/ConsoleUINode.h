/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once
#include <lutz/PodStates.h>
#include <string.h>
#include "ros/ros.h"
#include "lutz/UICode.h"
#include "lutz/UserInstructions.h"

namespace tsc_acs {
namespace lutz {

//! ROS Node to process Console Instructions.

class ConsoleUINode
{
public:
	//! Constructor
	ConsoleUINode();

	//! Executes node activity
	void spin();

private:
	void rxCallback(const ::lutz::UICode&  msg);

private:
	ros::NodeHandle node;
	ros::Subscriber sub;
	ros::Publisher pub;


	uint8_t displayState;
	uint8_t podState;
};

}
}
