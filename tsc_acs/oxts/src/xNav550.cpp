/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "oxts/xNav550Node.h"

//! xnav550 ROS node entry point.

//! Creates and spins a xNav550Node ROS node.
//! gps and utm are ROS parameters
//! that can be set using ROS; all are passed to the xNav550Node
//! constructor.
int main(int argc, char **argv)
{
    ros::init(argc, argv, "xnav550");
    ros::NodeHandle nodeParam("~");

    std::string frame_id = "gps";
    nodeParam.getParam("frame_id", frame_id);

    std::string frame_id_vel = "utm";
    nodeParam.getParam("frame_id_vel", frame_id_vel);

    tsc_acs::oxts::xNav550Node node(frame_id, frame_id_vel);
    node.spin();

    return 0;
}
