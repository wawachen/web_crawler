/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "acs/SteerSmootherNode.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "steer_smoother");
  tsc_acs::acs::SteerSmootherNode node;
  node.spin();
}
