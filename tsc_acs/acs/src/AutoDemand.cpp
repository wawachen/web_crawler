/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "acs/AutoDemandNode.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto_demand");
  tsc_acs::acs::AutoDemandNode node;
  node.spin();
}
