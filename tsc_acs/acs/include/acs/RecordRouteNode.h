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

class RecordRouteNode
{
public:
  RecordRouteNode();

  void spin();

  void gpsCallback(const oxts::BatchB & msg);

private:
  void checkSpacing();
  void addRecentToPath();
  bool isLoopClosed(double lat, double lng, double heading, GpsPath::const_iterator & start);
  void saveFile(GpsPath::const_iterator start);

private:
  ros::NodeHandle node;
  ros::Subscriber sub;

  GpsPath recent;

  GpsPath path;
  bool initialised;

  std::string pathfile;
  double waypointSpacing2;  // m2
  double waypointHeadingSpacing2;   // rad2
  double closureTolerance2;  // As fraction of spacing2
  double minimumLoop2;  // As multiple of spacing2

  int n, m;
};

}
}
