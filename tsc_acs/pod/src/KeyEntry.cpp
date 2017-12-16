/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "pod/KeyEntryNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "key_entry");

    tsc_acs::lutz::KeyEntryNode node;
    node.spin();

    return 0;
}
