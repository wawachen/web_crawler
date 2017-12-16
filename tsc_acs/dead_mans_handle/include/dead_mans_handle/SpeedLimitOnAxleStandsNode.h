/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "dead_mans_handle/SpeedLimit.h"


namespace tsc_acs {
namespace deadMansHandle {


class SpeedLimitOnAxleStandsNode
{

public:
	// Constructor
	SpeedLimitOnAxleStandsNode();

	void spin();

private:

	void publishSpeedLimit();

private:
	ros::NodeHandle node;
	ros::Rate sendRate;
	ros::Publisher pubSpeedLimit;

};




}
}
