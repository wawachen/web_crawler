/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "../include/lutz/ConsoleUINode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "console");

    tsc_acs::lutz::ConsoleUINode node;
    node.spin();

    return 0;
}
