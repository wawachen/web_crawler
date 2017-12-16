/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "acs/RecordRouteNode.h"
#include <math.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <cerrno>
#include <cstring>

#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include "../../version/version.h"

namespace tsc_acs {
namespace acs {

RecordRouteNode::RecordRouteNode()
  : initialised(false), n(0), m(0)
{
	ReportVersion(node);
  ros::NodeHandle nodeParam("~");
  nodeParam.getParam("pathfile", pathfile);
  if(pathfile.empty())
  {
    ROS_ERROR("pathfile not specified.");
  }
  double waypointSpacing = 0.1;
  nodeParam.getParam("waypoint_spacing", waypointSpacing);
  waypointSpacing2 = waypointSpacing*waypointSpacing;
  double waypointHeadingSpacing = 5 * 3.14/180;
  nodeParam.getParam("waypoint_heading_spacing", waypointHeadingSpacing);
  waypointHeadingSpacing2 = waypointHeadingSpacing*waypointHeadingSpacing;
  double closureTolerance = 1; // multiplier of waypoint spacing
  nodeParam.getParam("closure_tolerance", closureTolerance);
  closureTolerance2 = closureTolerance * closureTolerance;
  double minimumLoop = 50; // multiplier of waypoint spacing
  nodeParam.getParam("minimum_loop", minimumLoop);
  minimumLoop2 = minimumLoop * minimumLoop;

  sub = node.subscribe("/oxts/batchB", 300, &RecordRouteNode::gpsCallback, this);
}

void RecordRouteNode::spin()
{
  ros::spin();
}

void RecordRouteNode::gpsCallback(const oxts::BatchB & msg)
{
  if(!initialised)
  {
    path.Origin().Set(msg.latitude, msg.longitude, msg.heading, 0, 0, 0);
    recent.Origin().Set(msg.latitude, msg.longitude, msg.heading, 0, 0, 0);

    std::vector<double> originVector;
    originVector += path.Origin().Latitude(), path.Origin().Longitude(), path.Origin().Heading();
    originVector += path.Origin().X(), path.Origin().Y(), path.Origin().Theta();
    node.setParam("/acs/origin", originVector);

    initialised = true;
  }

  recent.Add(msg.latitude, msg.longitude, msg.heading);
  m++;

  checkSpacing();

  GpsPath::const_iterator start;
  if (isLoopClosed(msg.latitude, msg.longitude, msg.heading, start))
  {
    addRecentToPath();
    saveFile(start);
    ros::shutdown();
  }
}

void RecordRouteNode::saveFile(GpsPath::const_iterator start)
{
  std::ofstream file(pathfile.c_str());

  if (file.fail())
  {
    ROS_ERROR("Failed to save file (%s) : %s : %d", pathfile.c_str(), std::strerror(errno), errno);
  }

  file  << std::fixed << std::setprecision(7);

  for(GpsPath::const_iterator pnt=start;
      pnt != path.end(); pnt++)
  {
    file << pnt->Latitude() << ','
        << pnt->Longitude() << ','
        << pnt->Heading() << std::endl;
  }
}

void RecordRouteNode::checkSpacing()
{
  const GpsPathPoint & first = recent.front();
  const GpsPathPoint & last = recent.back();

  double dx = last.X() - first.X();
  double dy = last.Y() - first.Y();
  double dtheta = fmod(last.Theta() - first.Theta() + 3*M_PI, 2*M_PI) - M_PI;
  double s2 = (dx*dx + dy*dy) / waypointSpacing2 +
      dtheta*dtheta/waypointHeadingSpacing2;

  if (s2 > 1)
  {
    addRecentToPath();
  }
}

void RecordRouteNode::addRecentToPath()
{
  const GpsPathPoint & first = recent.front();
  const GpsPathPoint & last = recent.back();

  // Add average point to path
  double sumLat = 0;
  double sumLng = 0;
  double sumdHeading = 0;
  double heading0 = first.Heading();

  for(GpsPath::const_iterator pnt=recent.begin();
      pnt != recent.end(); pnt++)
  {
    sumLat += pnt->Latitude();
    sumLng += pnt->Longitude();
    sumdHeading += fmod((pnt->Heading() - heading0 + 3*M_PI), (2*M_PI))-M_PI;
  }

  path.Add(sumLat/recent.size(), sumLng/recent.size(),
           fmod(heading0 + sumdHeading/recent.size() + M_PI, 2*M_PI)-M_PI);

  recent.clear();

  GpsPath::const_reference newPnt = path.back();
  recent.Add(newPnt.Latitude(), newPnt.Longitude(), newPnt.Heading());
  ROS_INFO("Path point: %d, %d, (%.6f, %.6f, %.6f) [%.3f, %.3f, %.3f]",
           n++, m, newPnt.Latitude(), newPnt.Longitude(), newPnt.Heading(),
           newPnt.X(), newPnt.Y(), newPnt.Theta());
}

bool RecordRouteNode::isLoopClosed(double lat, double lng, double heading, GpsPath::const_iterator & start)
{
  double x, y, theta;
  path.Origin().ToXY(lat, lng, heading, x, y, theta);

  bool closePointFound = false;
  for(GpsPath::const_iterator pnt=path.begin();
      pnt != path.end(); pnt++)
  {
    double dx = x - pnt->X();
    double dy = y - pnt->Y();
    double dtheta = fmod(theta - pnt->Theta() + M_PI, 2*M_PI) - M_PI;
    double s2 = (dx*dx + dy*dy) / waypointSpacing2 +
        dtheta*dtheta/waypointHeadingSpacing2;
    if (!closePointFound)
    {
      if (s2 < closureTolerance2)
      {
       closePointFound = true;
       start = pnt;
      }
    }
    else if (s2 > minimumLoop2)
    {
      return true;
    }
  }

  return false;
}

}
}
