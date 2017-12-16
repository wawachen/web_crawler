/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include <pod/TurnigyJoyNode.h>

namespace tsc_acs {
namespace pod {

TurnigyJoyNode::TurnigyJoyNode(float slowSpeed, float maxSpeed, float maxReverse, float maxSteer,
		uint16_t deadZoneThrottle, uint16_t deadZoneSteer,
		uint16_t minStick, uint16_t maxStick, uint16_t centreStick, int16_t steerScalingFactor)
  : slowSpeed(slowSpeed), maxSpeed(maxSpeed), maxReverse(maxReverse), maxSteer(maxSteer),
    deadZoneThrottle(deadZoneThrottle), deadZoneSteer(deadZoneSteer),
	minStick(minStick), maxStick(maxStick), centreStick(centreStick), steerScalingFactor(steerScalingFactor)
{
	sub = node.subscribe("arduino/rc_in", 10, &TurnigyJoyNode::rxCallback, this);

	pubPodDemand = node.advertise< ::pod::PodDemandSource>("pod/turnigy_pod_demand", 10);
	pubAuxDemand = node.advertise< ::pod::AuxiliaryDemand>("pod/turnigy_auxiliary_demand", 10);
}

uint8_t TurnigyJoyNode::threePositionSwitch(int16_t value)
{
	  if (value < (minStick + (3 * (maxStick-minStick))) / 10)
	    return 0;
	  else if (value > minStick + 7 * (maxStick-minStick) / 10)
	    return 2;
	  else
	    return 1;
}

void TurnigyJoyNode::rxCallback(const arduino::rcReader & msg)
{
    ros::Duration age = ros::Duration(msg.age/1000000, (msg.age % 1000000)*1000);
    ros::Time msgTime = ros::Time::now() - age;

	::pod::PodDemandSource podDemand;
	::pod::AuxiliaryDemand auxDemand;

	podDemand.podDemand.header.stamp = msgTime;
	auxDemand.header.stamp = msgTime;

	//Channels mapping
	int16_t rightStick_LR 	= 	msg.ch1;
	int16_t rightStick_UD	= 	msg.ch2; //throttle
	int16_t leftStick_UD 	=	msg.ch3;
	int16_t leftStick_LR 	=	msg.ch4; //steer
	int16_t left_3pos_SW	= 	threePositionSwitch(msg.ch5); //abdicate
	int16_t right_3pos_SW	= 	threePositionSwitch(msg.ch6); //abdicate
	int16_t centralSwitch	= 	threePositionSwitch(msg.ch7); //drive by wire request
	int16_t centralDial		= 	msg.ch8; //speed limiter

	// creating speed factor using the centralDial
	float dialInput = ((centralDial-minStick)/(maxStick-minStick));// creating a factor from 0 to 1;
	float speedLimit = dialInput * (maxSpeed - slowSpeed) + slowSpeed;

	if(speedLimit >= maxSpeed)
	{
		speedLimit = maxSpeed;
	}
	else if(speedLimit <= slowSpeed)
	{
		speedLimit = slowSpeed;
	}

	///THROTTLE
    if (rightStick_UD > maxStick)
    {
      podDemand.podDemand.speed = speedLimit;
    }
    else if (rightStick_UD > centreStick + deadZoneThrottle)
	{
      podDemand.podDemand.speed = (rightStick_UD - centreStick - deadZoneThrottle) * speedLimit
                                      / (maxStick - centreStick - deadZoneThrottle);
	}
	else if (rightStick_UD < minStick)
	{
      podDemand.podDemand.speed = -maxReverse;
	}
	else if (rightStick_UD < centreStick - deadZoneThrottle)
	{
      podDemand.podDemand.speed = (rightStick_UD - centreStick + deadZoneThrottle) * maxReverse
                                      / (centreStick - deadZoneThrottle - minStick);
	}
	else
	{
	  podDemand.podDemand.speed = 0;
	}

	///STEER
    if (leftStick_LR > maxStick)
    {
      podDemand.podDemand.steer = -(float)maxSteer/ steerScalingFactor;
    }
    else if (leftStick_LR > centreStick + deadZoneSteer)
    {
      podDemand.podDemand.steer = -(float)(leftStick_LR - centreStick - deadZoneSteer) * maxSteer
                                      / (maxStick - centreStick - deadZoneSteer)
									  / steerScalingFactor;
    }
    else if (leftStick_LR < minStick)
    {
      podDemand.podDemand.steer = (float)maxSteer / steerScalingFactor;
    }
    else if (leftStick_LR < centreStick - deadZoneThrottle)
    {
      podDemand.podDemand.steer = -(float)(leftStick_LR - centreStick + deadZoneSteer) * maxSteer
                                      / (centreStick - deadZoneSteer - minStick)
									  / steerScalingFactor;
    }
    else
    {
      podDemand.podDemand.steer = 0;
    }

	podDemand.abdicate = left_3pos_SW == 1 && right_3pos_SW == 1;
	podDemand.podDemand.driveByWireRequest = centralSwitch == 2;

	controlSource.data = "TurnigyJoy";
	podDemand.podDemand.source = controlSource.data;

	//AUXILIARY DEMAND - set auxiliary demand here
	auxDemand.indicateLeft = left_3pos_SW == 2;
	auxDemand.indicateRight = right_3pos_SW == 2;
	auxDemand.hornOn = 0;
	auxDemand.mimicLights = 0;

	pubPodDemand.publish(podDemand);
	pubAuxDemand.publish(auxDemand);
}

void TurnigyJoyNode::spin()
{
	ros::spin();
}

}
}
