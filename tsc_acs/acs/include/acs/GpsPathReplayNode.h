/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "acs/Goal.h"
#include "std_msgs/Header.h"
#include "acs/GpsPath.h"
#include <dynamic_reconfigure/server.h>
#include <acs/PathReplayConfig.h>

namespace tsc_acs {
namespace acs {

class GpsPathReplayNode
{
public:
  GpsPathReplayNode();

  void spin();

  void locationCallback(const ::acs::Pose2DStamped & msg);
  void resetCallback(const std_msgs::Header & msg);
  void configCallback(::acs::PathReplayConfig & config, uint32_t level);
  double calculateLookAheadDistance(const ::acs::Goal & nextPoint);

private:
  ros::NodeHandle node;
  ros::Subscriber subLocation;
  ros::Subscriber subReset;
  ros::Publisher pub;
  ros::Publisher pub3;
  ros::Publisher pubPath;
  ros::Publisher pubConfig;

  GpsPath path;
  double loodAheadTime2; //s
  double minLookAheadDistance2;
  double maxLookAheadDistance2;
//  double lookAhead2; // m2

  bool reset;

  int nextGoalIndex;    // TODO: Consider using GpsPath::const_iterator

	dynamic_reconfigure::Server< ::acs::PathReplayConfig> configServer;
};

}
}
