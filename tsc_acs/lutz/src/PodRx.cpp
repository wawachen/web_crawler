/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pod_rx");

    tsc_acs::lutz::PodRxNode node;
    node.spin();

    return 0;
}
