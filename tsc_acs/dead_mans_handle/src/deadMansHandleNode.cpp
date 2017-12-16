/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "dead_mans_handle/deadMansHandleNode.h"
#include "dead_mans_handle/SpeedLimit.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace deadMansHandle {

deadMansHandleNode::deadMansHandleNode(float rangeMin, float rangeMax)
: rangeMin(rangeMin), rangeMax(rangeMax)
{
	ReportVersion(node);
	sub = node.subscribe("arduino/deadMansHandle", 10, &deadMansHandleNode::rxCallback, this);

	pubSpeedLimit = node.advertise< ::dead_mans_handle::SpeedLimit>("pod/speed_limit", 10);
}

void deadMansHandleNode::rxCallback(const arduino::deadMansHandleReading & msg)
{
	::dead_mans_handle::SpeedLimit speedLimit;
	speedLimit.header.stamp = msg.header.stamp;

	if (msg.analogReading < rangeMin)
	{
		speedLimit.speedLimit = 0.0;
	}
	else if (msg.analogReading > rangeMin && msg.analogReading < rangeMax)
	{
		speedLimit.speedLimit = (msg.analogReading - rangeMin)/(rangeMax - rangeMin);
	}
	else
	{
		speedLimit.speedLimit = 1.0;
	}

	pubSpeedLimit.publish(speedLimit);
}

void deadMansHandleNode::spin()
{
	ros::spin();
}

}
}

