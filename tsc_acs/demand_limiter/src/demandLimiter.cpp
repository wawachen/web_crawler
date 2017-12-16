/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "../include/demand_limiter/demandLimiterNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demand_limiter");

    ros::NodeHandle nodeParam("~");

    bool ratio = true;
    nodeParam.getParam("ratio", ratio);

    float maxSpeed = 24;
    nodeParam.getParam("maxSpeed", maxSpeed);

    tsc_acs::demandLimiter::demandLimiterNode node(ratio, maxSpeed);

    node.spin();

    return 0;
}



