/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "dead_mans_handle/SpeedLimitOnAxleStandsNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_limit_on_axle_stands");

    tsc_acs::deadMansHandle::SpeedLimitOnAxleStandsNode node;
    node.spin();

    return 0;
}



