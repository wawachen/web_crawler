/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "mobileye/MobileyeNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mobileye");

    tsc_acs::mobileye::MobileyeNode node;
    node.spin();

    return 0;
}
