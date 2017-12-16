/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "pod/PodDemandSource.h"
#include "pod/AuxiliaryDemand.h"
#include <std_msgs/String.h>

namespace tsc_acs {
namespace pod {

//! ROS Node for turning joystick input into pod demand.
class XboxJoyNode
{
public:
	//! Constructor
	XboxJoyNode(float slowSpeed, float maxSpeed, float maxReverse,
		  float deadZone, float maxSteer);

	//! Executes node activity
	void spin();

private:
  void rxCallback(const sensor_msgs::Joy & msg);

private:
	ros::NodeHandle node;

	ros::Subscriber sub;
	ros::Publisher pubPodDemand;
	ros::Publisher pubAuxDemand;

	float slowSpeed;
	float maxSpeed;
	float maxReverse;
	float deadZone;
	float maxSteer;
	bool driveByWireRequest;
	bool brakeOn;
	bool hazards;
	unsigned char wheelLights;

	sensor_msgs::Joy lastMsg;
	bool lastAbdicate;
	std_msgs::String controlSource;
};

}
}
