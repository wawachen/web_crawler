/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/OnAxleStandsNode.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace lutz {

OnAxleStandsNode::OnAxleStandsNode()
{
	ReportVersion(node);
	subPodDemand= node.subscribe("pod_demand", 10, &OnAxleStandsNode::demandCallback, this);
	pubPodDemand = node.advertise< ::pod::PodDemand>("pod/pod_demand_on_axle_stands", 10);
}

void OnAxleStandsNode::demandCallback(const ::pod::PodDemand & controlDemand)
{

	demand = controlDemand;
	demand.speed = 0.0;
	pubPodDemand.publish(demand);
}

void OnAxleStandsNode::spin()
{
	ros::spin();
}

}
}

