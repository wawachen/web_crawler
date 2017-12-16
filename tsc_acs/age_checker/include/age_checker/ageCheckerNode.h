/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include <ros/console.h>
#include "pod/PodDemand.h"


namespace tsc_acs {
namespace ageChecker {


class ageCheckerNode
{

public:
	// Constructor
	ageCheckerNode(int rate, float ageToStartRamp, float ageToStop);

	void spin();

private:

	void demandCallback(const ::pod::PodDemand & controlPodDemand);
	void checkAge();

private:
	ros::NodeHandle node;

	ros::Subscriber subPodDemandLimited;
	ros::Publisher pubPodDemand;
	::pod::PodDemand demand;

	ros::Rate rate;
	ros::Duration ageToStartRamp;
	ros::Duration ageToStop;

};




}
}
