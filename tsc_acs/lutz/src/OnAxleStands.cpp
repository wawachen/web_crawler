/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/OnAxleStandsNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "on_axle_stands");

    tsc_acs::lutz::OnAxleStandsNode node;
    node.spin();

    return 0;
}



