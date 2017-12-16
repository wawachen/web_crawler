/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "ibeo/ObjectMarker.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ibeo_object_marker");

    tsc_acs::ibeo::ObjectMarkerNode node;
    node.spin();

    return 0;
}


