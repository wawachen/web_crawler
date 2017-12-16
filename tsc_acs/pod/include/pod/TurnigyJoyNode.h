/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
//#include "mavros_msgs/RCIn.h"
#include "arduino/rcReader.h"
#include "pod/AuxiliaryDemand.h"
#include "pod/PodDemandSource.h"
#include <std_msgs/String.h>


namespace tsc_acs {
namespace pod {

//! ROS Node for turning joystick input into pod demand.
class TurnigyJoyNode
{
public:
	//! Constructor
	TurnigyJoyNode(float slowSpeed, float maxSpeed, float maxReverse, float maxSteer,
				uint16_t deadZoneThrottle, uint16_t deadZoneSteer,
				uint16_t minStick, uint16_t maxStick, uint16_t centreStick,
				int16_t ticksPerRadianPerM);

	//! Executes node activity
	void spin();

private:
  	void rxCallback(const arduino::rcReader & msg);
  	uint8_t threePositionSwitch(int16_t value);

private:
	ros::NodeHandle node;

	ros::Subscriber sub;
	ros::Publisher pubPodDemand;
	ros::Publisher pubAuxDemand;

	float slowSpeed;
	float maxSpeed;
	float maxReverse;
	uint16_t deadZoneThrottle;
	uint16_t deadZoneSteer;
	uint16_t maxSteer;
	uint16_t minStick;
	uint16_t maxStick;
	uint16_t centreStick;
	int16_t steerScalingFactor;

	std_msgs::String controlSource;
};

}
}
