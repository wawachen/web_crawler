/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "../include/age_checker/ageCheckerNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "age_checker");

    ros::NodeHandle nodeParam("~");

    int publishRate = 25;
    nodeParam.getParam("rate", publishRate);

    float ageToStartRamp = 0.2;	//secs
    nodeParam.getParam("age_to_start_ramp", ageToStartRamp);

    float ageToStop = 2;	//secs
    nodeParam.getParam("age_to_stop", ageToStop);

    tsc_acs::ageChecker::ageCheckerNode node(publishRate, ageToStartRamp, ageToStop);

    node.spin();

    return 0;
}



