/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "ibeo/FusionObjectList.h"
#include <visualization_msgs/MarkerArray.h>

namespace tsc_acs {
namespace ibeo {

//! Node for converting Ibeo objects into visualisable markers
class ObjectMarkerNode
{
public:
  ObjectMarkerNode();
  void spin();

private:
  void rxCallback(const ::ibeo::FusionObjectList & msg);

private:
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;

  visualization_msgs::MarkerArray markerArray;
                        // Member only to avoid continual reallocation
};

}
}
