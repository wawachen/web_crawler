/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "ros/ros.h"
#include "pod/PodDemandSource.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "o_demand");

  ros::NodeHandle nodeParam("~");
  ros::NodeHandle node;

  ros::Publisher pub = node.advertise< ::pod::PodDemandSource>("pod_demand", 2);
  ros::Rate rate (10);

  ::pod::PodDemandSource demand;
  demand.podDemand.steer = -0.01;
  demand.podDemand.speed = 0.1;
  demand.podDemand.source = "o_demand";
  demand.podDemand.driveByWireRequest = true;
  demand.abdicate = false;

  while(ros::ok())
  {
    demand.podDemand.header.stamp = ros::Time::now();
    pub.publish(demand);
    ros::spinOnce();
    rate.sleep();
  }
}
