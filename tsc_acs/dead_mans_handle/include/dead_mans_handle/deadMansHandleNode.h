/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include <ros/console.h>
#include "arduino/deadMansHandleReading.h"

namespace tsc_acs {
namespace deadMansHandle {

class deadMansHandleNode
{

public:
	// Constructor
	deadMansHandleNode(float rangeMin, float rangeMax);
	void spin();

private:
	void rxCallback(const arduino::deadMansHandleReading & msg);

private:
	ros::NodeHandle node;

	ros::Subscriber sub;
	ros::Publisher pubSpeedLimit;

	float rangeMin;
	float rangeMax;
};

}
}
