/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/ConsoleUINode.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace lutz {

ConsoleUINode::ConsoleUINode() :podState(0), displayState(0)
{
	ReportVersion(node);
	sub = node.subscribe("ui_code", 10, &ConsoleUINode::rxCallback, this);
}

void ConsoleUINode::rxCallback(const ::lutz::UICode& msg)
{
	if(podState != msg.pod_state)
	{
		podState = msg.pod_state;
		ROS_INFO("pod state: %s: %d", PodState[podState], podState);
	}
	if(displayState != msg.instruction)
	{
		ROS_INFO("%s",Instructions[msg.instruction]);
		displayState = msg.instruction;
	}

}

void ConsoleUINode::spin()
{
	while (ros::ok())
	{
		ros::spinOnce();
	}
}

}
}
