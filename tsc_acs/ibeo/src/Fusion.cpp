/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "ibeo/FusionNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion");

    ros::NodeHandle nodeParam("~");

    bool reportUnprocessedMessages = true;
    nodeParam.getParam("reportUnused", reportUnprocessedMessages);

    tsc_acs::ibeo::FusionNode node(reportUnprocessedMessages);
    node.spin();

    return 0;
}
