/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/MessageBuilderNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "message_builder");

    ros::NodeHandle nodeParam("~");

    int ticksPerRadianPerM = 207;
    nodeParam.getParam("ticksPerRadianPerM", ticksPerRadianPerM);

    int maxSpeed = 24;
    nodeParam.getParam("max_speed", maxSpeed);

    tsc_acs::lutz::MessageBuilderNode node((int16_t) ticksPerRadianPerM, (int8_t) maxSpeed);
    node.spin();

    return 0;
}
