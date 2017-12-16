/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "acs/GpsToLocalNode.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_to_local");

  ros::NodeHandle nodeParam("~");

  tsc_acs::acs::GpsToLocalNode node;

  node.spin();
}
