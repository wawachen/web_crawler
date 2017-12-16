/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "../include/demand_limiter/demandLimiterNode.h"
#include <algorithm>
#include "../../version/version.h"

namespace tsc_acs {
namespace demandLimiter {


demandLimiterNode::demandLimiterNode(bool ratio, float maxSpeed)
: ratio(ratio), maxSpeed(maxSpeed), speedLimit(0.0)
{
	ReportVersion(node);
	subControlDemand = node.subscribe("pod_demand", 10, &demandLimiterNode::demandCallback, this);
	subSpeedLimit = node.subscribe("speed_limit", 10, &demandLimiterNode::limitCallback, this);
	pubPodDemand = node.advertise< ::pod::PodDemand>("pod/pod_demand_limited", 10);
}

void demandLimiterNode::limitCallback(const ::dead_mans_handle::SpeedLimit & rxSpeedLimit)
{
	speedLimit = rxSpeedLimit.speedLimit;
	speedLimitTime = rxSpeedLimit.header.stamp;
}

void demandLimiterNode::demandCallback(const ::pod::PodDemand & controlDemand)
{
	demand = controlDemand;

	if (controlDemand.header.stamp > speedLimitTime)
	{
		demand.header.stamp = speedLimitTime;
	}

	if (ratio)
	{
		demand.speed = controlDemand.speed*speedLimit;
	}
	else
	{
		demand.speed = std::min(controlDemand.speed, maxSpeed*speedLimit);
	}

	pubPodDemand.publish(demand);
}

void demandLimiterNode::spin()
{
	ros::spin();
}

}
}

