/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "dead_mans_handle/SpeedLimitOnAxleStandsNode.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace deadMansHandle {

SpeedLimitOnAxleStandsNode::SpeedLimitOnAxleStandsNode() : sendRate(10)
{
	ReportVersion(node);
	pubSpeedLimit = node.advertise< ::dead_mans_handle::SpeedLimit>("pod/speed_limit", 10);
}


void SpeedLimitOnAxleStandsNode::publishSpeedLimit()
{
	::dead_mans_handle::SpeedLimit speedLimit;
	speedLimit.header.stamp = ros::Time::now();
	speedLimit.speedLimit = 1.0;
	pubSpeedLimit.publish(speedLimit);
}

void SpeedLimitOnAxleStandsNode::spin()
{
	while (ros::ok())
	{
		publishSpeedLimit();
		ros::spinOnce();
		sendRate.sleep();
	}
}


}
}

