/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/SessionControlNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "session_control");

    tsc_acs::lutz::SessionControlNode node;
    node.spin();

    return 0;
}
