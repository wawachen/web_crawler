/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "acs/SteerSmootherNode.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace acs {

SteerSmootherNode::SteerSmootherNode()
  : ticksPerRadianPerM(207), smoothingTicks(5), previousSteer(-1000), increasingChange(true)
{
	ReportVersion(node);

  ros::NodeHandle nodeParam("~");
  nodeParam.getParam("smoothing_ticks", smoothingTicks);

  pub = node.advertise< ::pod::PodDemandSource>("/acs/demand", 2);
  sub = node.subscribe("/acs/raw_demand", 3, &SteerSmootherNode::demandCallback, this);
}

void SteerSmootherNode::spin()
{
  ros::spin();
}

void SteerSmootherNode::demandCallback(const ::pod::PodDemandSource & msg)
{
  double deltaSteer = msg.podDemand.steer - previousSteer;
  int ticksChange = (int)(deltaSteer * ticksPerRadianPerM);
  ::pod::PodDemandSource demand = msg;

  bool limited = false;
  if ((increasingChange != (deltaSteer > 0)) && abs(ticksChange) <= smoothingTicks)
  {
    limited = true;
    demand.podDemand.steer = previousSteer;
  }
  else
  {
    previousSteer = demand.podDemand.steer;
    increasingChange = deltaSteer > 0;
  }
  ROS_INFO("-%6.4f %s %-6.4f %-6.4f %d", msg.podDemand.steer, limited?"YES":"NO ", demand.podDemand.steer, deltaSteer, ticksChange);
  pub.publish(demand);
}

}
}
