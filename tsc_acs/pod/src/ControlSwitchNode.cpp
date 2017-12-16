/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "pod/ControlSwitchNode.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace lutz {

DemandSubscriber::DemandSubscriber(ControlSwitchNode * node, std::string & topic, int index)
  : node(node),  index(index), topic(topic), abdicate(false)
{
}

DemandSubscriber::DemandSubscriber(const DemandSubscriber & other)
  : node(other.node), index(other.index), topic(other.topic), abdicate(other.abdicate)
{
}

void DemandSubscriber::subscribe()
{
  ROS_INFO("Subscribing: %s", topic.c_str());
  sub = node->node.subscribe(topic, 10, &DemandSubscriber::handler, this);
}

void DemandSubscriber::handler(const ::pod::PodDemandSource & msg)
{
  lastMsgTime = msg.podDemand.header.stamp;
  abdicate = msg.abdicate;
  node->messageReceived(msg, index);
}

bool DemandSubscriber::abdicated() const
{
  return abdicate &&
      ros::Time::now() - lastMsgTime < node->maxTimeBetweenMsgs;
}

ControlSwitchNode::ControlSwitchNode()
{
	ReportVersion(node);
  ros::NodeHandle nodeParam("~");

  std::string sources;
  nodeParam.getParam("sources", sources);
  double maxSecondsBetweenMsgs = 0.1;
  nodeParam.getParam("max_time_between_msgs", maxSecondsBetweenMsgs);
  maxTimeBetweenMsgs = ros::Duration(maxSecondsBetweenMsgs);

  std::istringstream sourcesStream(sources);
  std::string topic;
  while(getline(sourcesStream, topic, ';'))
  {
    ROS_INFO("Source: %s", topic.c_str());
    subscribers.push_back(DemandSubscriber(this, topic, subscribers.size()));
  }

  for(std::vector<DemandSubscriber>::iterator s = subscribers.begin();
      s != subscribers.end(); s++)
  {
      s->subscribe();
  }

  pub = node.advertise< ::pod::PodDemand >("pod/pod_demand", 10, true);
}

void ControlSwitchNode::spin()
{
  ros::spin();
}

void ControlSwitchNode::messageReceived(const ::pod::PodDemandSource & msg, int index)
{
  for(int i=0; i<index; i++)
    if (!subscribers[i].abdicated())
      return;   // A higher priority subscriber has control

  ::pod::PodDemand podDemand;

  if (!subscribers[index].abdicated())
  {
    pub.publish(msg.podDemand);
  }
  else if (index == subscribers.size() - 1) // All topics are abdicating
  {
    podDemand.header = msg.podDemand.header;
    podDemand.speed = 0;
    podDemand.steer = 0;
    podDemand.driveByWireRequest = false;
    podDemand.source = "null demand";
    pub.publish(podDemand);
  }
}

}}
