/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "pod/PodDemand.h"
#include "../../../acs/include/acs/GpsPath.h"

namespace tsc_acs {
namespace sim_pod {

class PodSimNode
{
public:
  PodSimNode(double steerAccel, double speedAccel);

  void spin();

  void demandCallback(const ::pod::PodDemand & msg);

private:
  ros::NodeHandle node;
  ros::Subscriber subDemand;
  ros::Publisher pubLocation;
  ros::Publisher pubLocation3;
  ros::Publisher pubGps;

	acs::GpsPathPoint origin;
	acs::GpsPathPoint start;

  geometry_msgs::Pose2D pose;
  ::pod::PodDemand previous;

  double speed;
  double steer;

  double steerAccel;    // rad/m/s
  double speedAccel;    // m/s2
};

}
}
