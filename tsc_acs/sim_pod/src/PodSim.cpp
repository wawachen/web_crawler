/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "sim_pod/PodSimNode.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pod_sim");

  ros::NodeHandle nodeParam("~");
  double steerAccel = 1;
  nodeParam.getParam("steer_accel", steerAccel);
  double speedAccel = 1;
  nodeParam.getParam("speed_accel", speedAccel);

  tsc_acs::sim_pod::PodSimNode node(steerAccel, speedAccel);

  node.spin();
}
