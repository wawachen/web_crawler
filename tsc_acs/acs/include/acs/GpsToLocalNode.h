/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "acs/GpsPath.h"
#include "oxts/BatchB.h"

namespace tsc_acs {
namespace acs {

class GpsToLocalNode
{
public:
  GpsToLocalNode();

  void spin();

  void gpsCallback(const oxts::BatchB & msg);

private:
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;

  GpsPathPoint origin;
};

}
}
