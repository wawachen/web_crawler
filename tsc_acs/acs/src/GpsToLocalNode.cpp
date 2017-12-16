/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "acs/GpsToLocalNode.h"
#include "acs/Pose2DStamped.h"
#include <vector>
#include <unistd.h>
#include "../../version/version.h"

namespace tsc_acs {
namespace acs {

GpsToLocalNode::GpsToLocalNode()
{
	ReportVersion(node);
  sub = node.subscribe("batchB", 3, &GpsToLocalNode::gpsCallback, this);
  pub = node.advertise< ::acs::Pose2DStamped>("/acs/location", 2);

  while(!node.hasParam("/acs/origin"))
  {
    ROS_INFO("Waiting for /acs/origin");
    usleep(1000000);    // Sleep 1s
  }
  std::vector<double> origin_in;
  node.getParam("/acs/origin", origin_in);
  ROS_INFO("Origin %2.4f, %2.4f, %2.4f : %2.4f, %2.4f, %2.4f",
           origin_in[0], origin_in[1], origin_in[2],
           origin_in[3], origin_in[4], origin_in[5]);

  origin.Set(origin_in[0], origin_in[1], origin_in[2],
             origin_in[3], origin_in[4], origin_in[5]);
}

void GpsToLocalNode::spin()
{
  ros::spin();
}

void GpsToLocalNode::gpsCallback(const oxts::BatchB & msg)
{
  ::acs::Pose2DStamped location;
  location.header.stamp = msg.header.stamp;

  origin.ToXY(msg.latitude, msg.longitude, msg.heading,
              location.pose.x, location.pose.y, location.pose.theta);
  pub.publish(location);

}

}
}
