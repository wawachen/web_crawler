/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include <pod/XboxJoyNode.h>

//! pod_joy ROS node entry point.

//! Creates and spins a Drive by Joystick ROS node.
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pod_xbox_joy");

    ros::NodeHandle nodeParam("~");

    float slowSpeed = 12;
    nodeParam.getParam("slow_speed", slowSpeed);

    float maxSpeed = 12;
    nodeParam.getParam("max_speed", maxSpeed);

    float maxReverse = 5;
    nodeParam.getParam("max_reverse", maxReverse);

    float deadZone = 0.1f;
    nodeParam.getParam("max_reverse", deadZone);

    float maxSteer = 100;
    nodeParam.getParam("max_steer", maxSteer);

    tsc_acs::pod::XboxJoyNode node(slowSpeed, maxSpeed, maxReverse, deadZone, maxSteer);

    node.spin();

    return 0;
}
