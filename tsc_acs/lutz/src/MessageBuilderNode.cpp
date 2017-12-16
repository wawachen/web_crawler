/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/MessageBuilderNode.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace lutz {

MessageBuilderNode::MessageBuilderNode(int16_t ticksPerRadianPerM, int8_t maxSpeed)
	: sendRate(20), maxSpeed(maxSpeed), podStateRequest(0), podState(0),
	  ticksPerRadianPerM(ticksPerRadianPerM)
{
	ReportVersion(node);
	subSession = node.subscribe("session_control", 10, &MessageBuilderNode::rxSessionControl, this);
	subPodDemand = node.subscribe("pod_demand", 10, &MessageBuilderNode::rxPodDemand, this);
	subAuxDemand = node.subscribe("pod/auxiliary_demand", 10, &MessageBuilderNode::rxAuxDemand, this);
	subPOD_STATUS = node.subscribe("status", 10, &MessageBuilderNode::handlePOD_STATUS, this);

	pubACS_AUX_POWER_REQ = node.advertise< ::lutz::AuxPowerRequest>("aux_power_request", 10, true);
	pubACS_CTRL_CMD1 = node.advertise< ::lutz::ControlCommand1>("control_command1", 10, true);
	pubACS_CTRL_CMD2 = node.advertise< ::lutz::ControlCommand2>("control_command2", 10, true);
}

void MessageBuilderNode::rxSessionControl(const ::lutz::SessionControl & msg)
{
	podStateRequest = msg.pod_state_request;
}

void MessageBuilderNode::rxPodDemand(const ::pod::PodDemand & msg)
{
	podDemand = msg;
}

void MessageBuilderNode::rxAuxDemand(const ::pod::AuxiliaryDemand & msg)
{
	auxDemand = msg;
}

void MessageBuilderNode::handlePOD_STATUS(const ::lutz::Status & msg)
{
	podState = msg.pod_state;
}

void MessageBuilderNode::buildMessages()
{
	controlCommand1.header = podDemand.header;
	controlCommand1.pod_state_request = podStateRequest;
	controlCommand1.max_speed = maxSpeed;
	controlCommand1.speed_mode = 1;
	controlCommand1.torque_limit = 100;

	if (podStateRequest == AUTONOMOUS and podState != AUTONOMOUS)
	{
		controlCommand1.direction = 1;
		controlCommand1.park_brake_off = 0;
		controlCommand1.throttle_pot = 0;
	}
	if (podState == AUTONOMOUS)
	{
		float speedDemandKph = podDemand.speed*3.6; //convert to kph
		if (podDemand.speed > 0)
		{
			controlCommand1.park_brake_off = 1;
			controlCommand1.direction = 1;
			controlCommand1.throttle_pot = (speedDemandKph *100)/maxSpeed;
		}
		else if (podDemand.speed < 0)
		{
			controlCommand1.park_brake_off = 1;
			controlCommand1.direction = 3;
			controlCommand1.throttle_pot = -(speedDemandKph *100)/maxSpeed;
		}
		else
		{
			controlCommand1.park_brake_off = 0;
			controlCommand1.throttle_pot = 0;
		}

		controlCommand2.header = podDemand.header;
		// pod demand steer takes anti-clockwise as positive, steer angle is right positive
		controlCommand2.front_steer_angle = podDemand.steer * -ticksPerRadianPerM;

		auxPowerRequest.header = auxDemand.header;
		auxPowerRequest.left_indicator = auxDemand.indicateLeft;
		auxPowerRequest.right_indicator = auxDemand.indicateRight;
		auxPowerRequest.horn = auxDemand.hornOn;
		auxPowerRequest.mimic_light = auxDemand.mimicLights;
	}
}

void MessageBuilderNode::publishMessages()
{
	pubACS_AUX_POWER_REQ.publish(auxPowerRequest);
	pubACS_CTRL_CMD1.publish(controlCommand1);
	pubACS_CTRL_CMD2.publish(controlCommand2);
}

void MessageBuilderNode::spin()
{
	while(ros::ok())
	{
		buildMessages();
		publishMessages();
		ros::spinOnce();
		sendRate.sleep();
	}
}

}
}
