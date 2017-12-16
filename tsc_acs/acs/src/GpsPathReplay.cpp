/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "acs/GpsPathReplayNode.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_path_replay");
  tsc_acs::acs::GpsPathReplayNode node;
  node.spin();
}
