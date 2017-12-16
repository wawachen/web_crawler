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
namespace lutz {


class OnAxleStandsNode
{

public:
	// Constructor
	OnAxleStandsNode();

	void spin();

private:

	void demandCallback(const ::pod::PodDemand & controlPodDemand);

private:
	ros::NodeHandle node;

	ros::Subscriber subPodDemand;
	ros::Publisher pubPodDemand;
	::pod::PodDemand demand;

};




}
}
