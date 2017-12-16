/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "pod/ControlSwitchNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_switch");
    tsc_acs::lutz::ControlSwitchNode node;
    node.spin();
    return 0;
}
