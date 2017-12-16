/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "../include/age_checker/ageCheckerNode.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace ageChecker {

ageCheckerNode::ageCheckerNode(int rate, float ageToStartRamp, float ageToStop)
: rate(rate), ageToStartRamp(ageToStartRamp), ageToStop(ageToStop)
{
	ReportVersion(node);
	demand.header.stamp = ros::Time(0,0);
	subPodDemandLimited = node.subscribe("pod_demand", 10, &ageCheckerNode::demandCallback, this);
	pubPodDemand = node.advertise< ::pod::PodDemand>("pod/pod_demand_age_checked", 10);
}


void ageCheckerNode::demandCallback(const ::pod::PodDemand & controlDemand)
{
	demand = controlDemand;
	ros::Duration age = ros::Time::now() - demand.header.stamp;
	if (age <= ageToStartRamp)
	{
		pubPodDemand.publish(demand);
	}
}

void ageCheckerNode::checkAge()
{
	ros::Duration age = ros::Time::now() - demand.header.stamp;

	if (age > ageToStartRamp)
	{
		::pod::PodDemand newDemand;
		newDemand.header = demand.header;
		newDemand.source = demand.source;
		newDemand.driveByWireRequest = demand.driveByWireRequest;

		if (age < ageToStop)
		{
			newDemand.speed = demand.speed * (ageToStop.toSec() - age.toSec()) /
					(ageToStop.toSec() - ageToStartRamp.toSec());
			newDemand.source += ": Age Checked";
		}
		else
		{
			newDemand.speed = 0.0;
			newDemand.steer = 0.0;
			newDemand.source += ": Age Checked (Zero)";
		}

		pubPodDemand.publish(newDemand);
	}
}


void ageCheckerNode::spin()
{

	while (ros::ok())
	{
		checkAge();
		ros::spinOnce();
		rate.sleep();
	}
}


}
}

