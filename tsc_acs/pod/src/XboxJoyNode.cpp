/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include <pod/XboxJoyNode.h>
#include "../../version/version.h"

namespace xbox
{
	enum axes {
		LSRL,
		LSUD,
		RSRL,
		RSUD,
		RT,
		LT,
		DRL,
		DUD
	};

	enum buttons {
		A, B, X, Y,
		RB, LB,
		Back, Start, Guide,
		LS, RS
	};
}

namespace tsc_acs {
namespace pod {

XboxJoyNode::XboxJoyNode(float slowSpeed, float maxSpeed, float maxReverse, float deadZone, float maxSteer)
  : slowSpeed(slowSpeed), maxSpeed(maxSpeed), maxReverse(maxReverse),
    deadZone(deadZone), maxSteer(maxSteer), lastAbdicate(false),
	driveByWireRequest(false), brakeOn(false), hazards(false), wheelLights(0)
{
	ReportVersion(node);
	sub = node.subscribe("joy", 10, &XboxJoyNode::rxCallback, this);

	pubPodDemand = node.advertise< ::pod::PodDemandSource>("pod/xbox_pod_demand", 10);
	pubAuxDemand = node.advertise< ::pod::AuxiliaryDemand>("pod/xbox_auxiliary_demand", 10);
}

void XboxJoyNode::rxCallback(const sensor_msgs::Joy & msg)
{
    ros::Time now = ros::Time::now();
	::pod::PodDemandSource podDemand;
	podDemand.podDemand.header.stamp = now;
	::pod::AuxiliaryDemand auxDemand;
	auxDemand.header.stamp = now;

	// Rescale RT and LT axes (1->-1) to (0 to 1).
	float rt = (1 - msg.axes[xbox::RT]) / 2;
	float lt = (1 - msg.axes[xbox::LT]) / 2;

	// If both pressed, speed = 0
	// Other use non-zero trigger
//	float topSpeed = msg.buttons[xbox::A] ? maxSpeed : slowSpeed;
//	podDemand.podDemand.speed = (lt == 0) ? topSpeed * rt :
//								((rt == 0) ? -maxReverse * lt : 0);
	ROS_INFO("SPEED : %3.2f",podDemand.podDemand.speed);

	if(msg.axes[xbox::RSUD] >= 0)
	{
		podDemand.podDemand.speed = maxSpeed * msg.axes[xbox::RSUD];
	}
	else if(msg.axes[xbox::RSUD] < 0)
	{
		podDemand.podDemand.speed = maxReverse * msg.axes[xbox::RSUD];
	}
	if(abs(podDemand.podDemand.speed) < 2)
	{
		podDemand.podDemand.speed = 0;
	}
	ROS_INFO("SPEED : %3.2f",podDemand.podDemand.speed);


	podDemand.podDemand.steer = -maxSteer * msg.axes[xbox::LSRL];
	ROS_INFO("STEER : %3.1f", podDemand.podDemand.steer);

	podDemand.abdicate = lastAbdicate;
	if(msg.buttons[xbox::Start])
	{
		podDemand.abdicate = true;
	}
	if(msg.buttons[xbox::Back])
	{
		podDemand.abdicate = false;
	}
	lastAbdicate = podDemand.abdicate;

	controlSource.data = "xbox";
	podDemand.podDemand.source = controlSource.data;
	ROS_INFO("SOURCE : %s", podDemand.podDemand.source.c_str());


	if(msg.buttons[xbox::Guide] && !lastMsg.buttons[xbox::Guide])
	{
		driveByWireRequest = !driveByWireRequest;
	}
	podDemand.podDemand.driveByWireRequest = driveByWireRequest;


	if (msg.axes[xbox::DRL] < -0.5f)
	{
		wheelLights++;
	}
	else if (msg.buttons[xbox::DRL] > 0.5f)
	{
		wheelLights--;
	}
	auxDemand.mimicLights = wheelLights;

	auxDemand.hornOn = msg.buttons[xbox::X];

	if (msg.buttons[xbox::Y] && !lastMsg.buttons[xbox::Y])
	{
		hazards = !hazards;
	}

	auxDemand.indicateLeft = hazards || msg.buttons[xbox::LB];
	auxDemand.indicateRight = hazards || msg.buttons[xbox::RB];

	lastMsg = msg;

	pubPodDemand.publish(podDemand);
	pubAuxDemand.publish(auxDemand);
}

void XboxJoyNode::spin()
{
	ros::spin();
}

}
}
