/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/SessionControlNode.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace lutz {

SessionControlNode::SessionControlNode()
  : frameCount(0), sessionId(0), sendRate(20),
	podStateRequest(0), autoRequest(0), podState(0),
	frontBumper(0), rearBumper(0), laseDriveByWireRequest(0), tempReadyState(ZERO_STATE), tempReadyStateOld(ZERO_STATE)
{
	ReportVersion(node);
	subPOD_HANDSHAKE = node.subscribe("pod_handshake", 10, &SessionControlNode::handlePOD_HANDSHAKE, this);
	subPOD_BATTERY = node.subscribe("battery", 10, &SessionControlNode::handlePOD_BATTERY, this);
	subPOD_EPS = node.subscribe("eps", 10, &SessionControlNode::handlePOD_EPS, this);
	subPOD_EPS_INBOARD = node.subscribe("eps_inboard", 10, &SessionControlNode::handlePOD_EPS_INBOARD, this);
	subPOD_EPS_OUTBOARD = node.subscribe("eps_outboard", 10, &SessionControlNode::handlePOD_EPS_OUTBOARD, this);
	subPOD_POWERTRAIN = node.subscribe("powertrain", 10, &SessionControlNode::handlePOD_POWERTRAIN, this);
	subPOD_PARK_BRAKE = node.subscribe("epb", 10, &SessionControlNode::handlePOD_PARK_BRAKE, this);
	subACS_CTRL_CMD1 = node.subscribe("control_command1", 10, &SessionControlNode::handleACS_CTRL_CMD1, this);
	subPOD_STATUS = node.subscribe("status", 10, &SessionControlNode::handlePOD_STATUS, this);
	subAutonomous_Request = node.subscribe("pod/pod_demand_limited", 10, &SessionControlNode::rxCallback, this);

	pubSession_Control = node.advertise< ::lutz::SessionControl>("session_control", 10, true);
	pubUICode = node.advertise< ::lutz::UICode>("ui_code", 10, true);
}

void SessionControlNode::rxCallback(const ::pod::PodDemand & msg)
{
	laseDriveByWireRequest = msg.driveByWireRequest;
	if(autoRequest != laseDriveByWireRequest)
	{
		if(laseDriveByWireRequest)
		{
			generateUserInstruction(DRIVE_BY_WIRE_REQ, true);
		}
		else
		{
			generateUserInstruction(DRIVE_BY_WIRE_CANCEL, true);
			MakeSessionCancelled();
		}
	}
}

void SessionControlNode::generateUserInstruction(const uint8_t uiTemp, const bool flag)
{
	::lutz::UICode msgOut;
	msgOut.instruction = uiTemp;
	msgOut.pod_state = statusMsg.pod_state;
	msgOut.flag = flag;
	pubUICode.publish(msgOut);

	ROS_INFO("%s", Instructions[uiTemp]);

	sendRate.sleep(); //sleep allows the console UI node to catch up.
}

void SessionControlNode::handlePOD_STATUS(const ::lutz::Status & msg)
{
	statusMsg = msg;
	handleBumpStrips();

	if(podState != statusMsg.pod_state or autoRequest != laseDriveByWireRequest or tempReadyState != tempReadyStateOld)
	{
		ROS_INFO("pod state: %s: %d", PodState[statusMsg.pod_state], statusMsg.pod_state);
		autoRequest = laseDriveByWireRequest;

		if(autoRequest)
		{
			tempReadyStateOld = tempReadyState;
			EnterAutonomous();
		}
		else
		{
			MakeSessionCancelled();
			if(statusMsg.pod_state == OVERRIDE)
			{
				tempReadyState = GetOverrideTrueState();
				GetOverrideTrueReason(tempReadyState);
				MakeOverrideFalse(tempReadyState);
			}
		}
		podState = statusMsg.pod_state;
	}
}

// handles the bump strips,
// as the bump strip resets itself after being activated,
// a flag needs to be raised so that it can be tracked
void SessionControlNode::handleBumpStrips()
{
	if(statusMsg.front_bumper)
	{
		frontBumper = statusMsg.front_bumper;
	}
	if(statusMsg.rear_bumper)
	{
		rearBumper = statusMsg.rear_bumper;
	}
	if(podState != E_STOP)
	{
		frontBumper = statusMsg.front_bumper;
		rearBumper = statusMsg.rear_bumper;
	}
}

void SessionControlNode::handlePOD_HANDSHAKE(const ::lutz::Handshake & msg)
{
	handshakeMsg = msg;
}

void SessionControlNode::handlePOD_BATTERY(const ::lutz::Battery & msg)
{
	batteryMsg = msg;
}

void SessionControlNode::handlePOD_EPS(const ::lutz::EPS & msg)
{
	epsMsg = msg;
}

void SessionControlNode::handlePOD_EPS_INBOARD(const ::lutz::Steer & msg)
{
	epsInboardMsg = msg;
}

void SessionControlNode::handlePOD_EPS_OUTBOARD(const ::lutz::Steer & msg)
{
	epsOutboardMsg = msg;
}

void SessionControlNode::handlePOD_POWERTRAIN(const ::lutz::Powertrain & msg)
{
	powertrainMsg = msg;
}

void SessionControlNode::handlePOD_PARK_BRAKE(const ::lutz::ParkBrake & msg)
{
	parkBrakeMsg = msg;
}

void SessionControlNode::handleACS_CTRL_CMD1(const ::lutz::ControlCommand1 & msg)
{
	controlCommand1Msg = msg;
}

void SessionControlNode::handlePublisher()
{
	::lutz::SessionControl msgOut;
	msgOut.pod_state_request = podStateRequest;
	msgOut.session_id = sessionId;
	pubSession_Control.publish(msgOut);
}

void SessionControlNode::spin()
{
	while (ros::ok())
	{
		handlePublisher();
		ros::spinOnce();
		sendRate.sleep();
	}
}

}
}
