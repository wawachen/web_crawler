/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "acs/RecordRouteNode.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "record_route");
  tsc_acs::acs::RecordRouteNode node;
  node.spin();
}
