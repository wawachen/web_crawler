/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include <pod/TurnigyJoyNode.h>

//! pod_mavJoy ROS node entry point.

//! Creates and spins a Drive by Joystick ROS node.
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pod_turnigy_Joy");

    ros::NodeHandle nodeParam("~");

    float slowSpeed = 5.0/3.60; // in mps;
    nodeParam.getParam("slow_speed", slowSpeed);

    float maxSpeed = 24.0/3.60; // in mps
    nodeParam.getParam("max_speed", maxSpeed);

    float maxReverse = 5.0/3.60; // in mps
    nodeParam.getParam("max_reverse", maxReverse);

    int deadZoneThrottle = 100;
    nodeParam.getParam("dead_zone_throttle", deadZoneThrottle);

    int deadZoneSteer = 20;
    nodeParam.getParam("dead_zone_steer", deadZoneSteer);

    int maxSteer = 100;
    nodeParam.getParam("max_steer", maxSteer);

    int minStick = 0;
    nodeParam.getParam("min_stick", minStick);

    int maxStick = 1000;
    nodeParam.getParam("max_stick", maxStick);

    int centreStick = 500;
    nodeParam.getParam("centre_stick", centreStick);

    int steerScalingFactor = 207;
    nodeParam.getParam("steerScalingFactor", steerScalingFactor);

    tsc_acs::pod::TurnigyJoyNode node(slowSpeed, maxSpeed, maxReverse, maxSteer,
                                      (uint16_t)deadZoneThrottle, (uint16_t)deadZoneSteer,
                                      (uint16_t)minStick, (uint16_t)maxStick, (uint16_t)centreStick,
									  (int16_t) steerScalingFactor);

    node.spin();

    return 0;
}
