/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include <ros/console.h>
#include "dead_mans_handle/SpeedLimit.h"
#include "pod/PodDemand.h"


namespace tsc_acs {
namespace demandLimiter {

class demandLimiterNode
{

public:
	// Constructor
	demandLimiterNode(bool ratio, float maxSpeed);
	void spin();

private:
	void demandCallback(const ::pod::PodDemand & controlPodDemand);
	void limitCallback(const ::dead_mans_handle::SpeedLimit & speedLimit);


private:
	ros::NodeHandle node;

	ros::Subscriber subControlDemand;
	ros::Subscriber subSpeedLimit;
	ros::Publisher pubPodDemand;

	::pod::PodDemand demand;

	bool ratio;
	float maxSpeed;
	float speedLimit;
	ros::Time speedLimitTime;
};

}
}
