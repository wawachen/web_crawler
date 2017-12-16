/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "pod/PodDemandSource.h"

namespace tsc_acs {
namespace acs {

class SteerSmootherNode
{
public:
  SteerSmootherNode();

  void spin();

  void demandCallback(const ::pod::PodDemandSource & msg);

private:
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;

  double ticksPerRadianPerM;   // m/rad
  int smoothingTicks;

  double previousSteer;
  bool increasingChange;
};

}
}
