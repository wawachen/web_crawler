/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "pod/PodDemandSource.h"
#include <std_msgs/String.h>
#include <vector>

namespace tsc_acs {
namespace lutz {

class ControlSwitchNode;

class DemandSubscriber
{
public:
  DemandSubscriber(ControlSwitchNode * node, std::string & topic, int priority);
  DemandSubscriber(const DemandSubscriber & other);

  void subscribe();

  bool abdicated() const;
  const std::string & topicName() const {return topic;}

private:
  void handler(const ::pod::PodDemandSource & msg);

private:
  ros::Subscriber sub;
  int index;
  std::string topic;
  bool abdicate;
  ros::Time lastMsgTime;

  ControlSwitchNode * node;
};

class ControlSwitchNode
{
public:
	ControlSwitchNode();
	void spin();

private:
	void messageReceived(const ::pod::PodDemandSource & msg, int index);

private:
	std::vector<DemandSubscriber> subscribers;

	ros::NodeHandle node;
	ros::Publisher pub;

	ros::Duration maxTimeBetweenMsgs;

	friend class DemandSubscriber;
};
}}
