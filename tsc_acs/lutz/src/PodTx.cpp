/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodTxNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pod_tx");

    tsc_acs::lutz::PodTxNode node;
    node.spin();

    return 0;
}
