/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include "lutz/PodHeader.h"
#include "lutz/Handshake.h"
#include "lutz/PodCanId.h"
#include "lutz/SessionControl.h"


namespace tsc_acs {
namespace lutz {


class HandshakeNode
{
public:
  //! Constructor
  //! Subscribes to the pod_handshake topic and advertises acs_handshake topic.
	  HandshakeNode();

  //! Executes node activity
  void spin();

private:
  void rxCallback(const ::lutz::Handshake & msg);
//  void rxSessionControl(const ::lutz::SessionControl & msg);
  void Handshake();

private:
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Subscriber subSessionControl;
  ros::Publisher pubACS_HANDSHAKE;

  uint64_t frameCount;
  uint64_t sessionId;

  ros::Rate sendRate;
  const ros::Duration qosTimeout;
  int8_t qos;
  uint8_t seed;
  ros::Time lastRx;
  uint8_t podSeed;
//  bool autoRequest;

  ::lutz::Handshake msgHandshakeIn;
};

}
}
