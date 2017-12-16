/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include "mobileye/ObstacleStatus.h"

namespace tsc_acs {
namespace mobileye {

//! ROS Node for interpreting to Mobileye CAN data.
class MobileyeNode
{
public:
  MobileyeNode();
  void spin();
private:
  void rxCallback(const can_msgs::Frame& msg);
  void handleObstacleStatus(const can_msgs::Frame& msg);
private:
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pubObstacleStatus;
};

}
}
