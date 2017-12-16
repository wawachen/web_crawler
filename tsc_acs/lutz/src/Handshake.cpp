/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/HandshakeNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "handshake");

    tsc_acs::lutz::HandshakeNode node;
    node.spin();

    return 0;
}
