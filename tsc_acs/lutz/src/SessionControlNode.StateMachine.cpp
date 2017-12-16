/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/SessionControlNode.h"

namespace tsc_acs {
namespace lutz {

/////////////////////////////////////////////////////////////////////
//KEY_MAN TRUE if
//OFF --> MANNUAL
/////////////////////////////////////////////////////////////////////
bool SessionControlNode::MakeKeyManual()
{
	bool checkState = false;
	if(batteryMsg.ignition_manual)
	{
		checkState = true;
	}
	else if(not batteryMsg.ignition_manual)
	{
		generateUserInstruction(KEY_ON, true);
	}
	return checkState;
}

/////////////////////////////////////////////////////////////////////
//KEY_AUTO TRUE if
//MANNUAL --> NO_SESSION
/////////////////////////////////////////////////////////////////////
bool SessionControlNode::MakeKeyAuto()
{
	bool checkState = false;
	if(batteryMsg.ignition_auto)
	{
		checkState = true;
	}else
	{
		generateUserInstruction(KEY_NOT_IN_AUTO, true);
	}
	return checkState;
}

/////////////////////////////////////////////////////////////////////
//SESSION = 0 if
//NO SESSION --> NEW_SESSION
//STATE REQ = NEW_SESSION AND ACS_CTRL_CMD1:SESSION_ID = 0 AND ACS_HANDSHAKE:SESSION_ID = 0
/////////////////////////////////////////////////////////////////////
void SessionControlNode::MakeSession0()
{
	sessionId = 0;
	podStateRequest = NEW_SESSION;
}

/////////////////////////////////////////////////////////////////////
//SESSION VALID if
//NEWSESSION --> STEERING CENTRING
//STATE REQ = PRIMED AND
//AUTONOMY READY and
//<User applies brake>
/////////////////////////////////////////////////////////////////////
ReadyState SessionControlNode::GetSessionValidState()
{
	ReadyState state = ZERO_STATE;
	state = (ReadyState)(state | GetAutonomyReadyState());
	if(sessionId == 0)
	{
		state = (ReadyState)(state | SESSION_ID_ERROR);
	}
	if(podStateRequest != PRIMED)
	{
		state = (ReadyState)(state | STATE_REQ_ERROR);
	}
	if(epsMsg.brake_pedal_force < BRAKE_PEDAL_FORCE_UPPER)
	{
		state = (ReadyState)(state | APPLY_BRAKE_ERROR);
	}
	if(state == 0x521)
	{
		state = SESSION_VALID;
	}
	return state;
}

void SessionControlNode::MakeSessionValid(const ReadyState & reason)
{
	if((reason & AUTONOMY_READY) == 0)
	{
		MakeAutonomyReady(reason);
	}
	else if((reason & SESSION_ID_ERROR) != 0)
	{
		ROS_INFO("setting session_id =1");
		sessionId = 1;
	}
	else if((reason & STATE_REQ_ERROR) != 0)
	{
		ROS_INFO("setting podStateRequest = PRIMED");
		podStateRequest = PRIMED;
	}
	else if((reason & APPLY_BRAKE_ERROR) != 0)
	{
		generateUserInstruction(BPL_LOW, true);
	}
}

/////////////////////////////////////////////////////////////////////
//PRIMED VALID if
//STEERING CENTRING --> PRIMED
//STATE REQ = PRIMED AND
//AUTONOMY READY AND
//<Steering wheel locked in 12o’clock position>
/////////////////////////////////////////////////////////////////////
ReadyState SessionControlNode::GetPrimedValidState()
{
	ReadyState state = ZERO_STATE;
	state = (ReadyState)(state | GetAutonomyReadyState());
	if(podStateRequest != PRIMED)
	{
		state = (ReadyState)(state | STATE_REQ_ERROR);
	}
	if(epsInboardMsg.steer_wheel_position != 1
			or epsOutboardMsg.steer_wheel_position != 1)
	{
		state = (ReadyState)(state | STEERING_CENTRING_ERROR);
	}
	if(state == 0x521)
	{
		state = PRIMED_VALID;
	}
	return state;
}

void SessionControlNode::MakePrimedValid(const ReadyState & reason)
{
	if((reason & AUTONOMY_READY) == 0)
	{
		MakeAutonomyReady(reason);
	}
	else if((reason & STATE_REQ_ERROR) != 0)
	{
		podStateRequest = PRIMED;
	}
	else if((reason & STEERING_CENTRING_ERROR) != 0)
	{
		generateUserInstruction(CENTRE_STEERING, true);
	}
}

/////////////////////////////////////////////////////////////////////
//AUTONOMOUS VALID if
//PRIMED --> AUTONOMOUS
//STATE REQ = AUTONOMOUS AND
//AUTONOMY READY AND
//ACS_CTRL_CMD1:DIRECTION != REV(3) AND
//ACS_CTRL_CMD1:MAX_SPEED <= 24 kph AND
//ACS_CTRL_CMD1:SPEED_MODE_REQ = 1 AND
//ACS_CTRL_CMD1:TORQUE_LIMIT = 100% AND
//ACS_CTRL_CMD1:PARK_BRAKE_OFF_REQ = 0 AND ACS_CTRL_CMD1:THROTTLE_POT= 0
/////////////////////////////////////////////////////////////////////
ReadyState SessionControlNode::GetAutonomousValidState()
{
	ReadyState state = ZERO_STATE;
	state = (ReadyState)(state| GetAutonomyReadyState());

	if(podStateRequest != AUTONOMOUS)
	{
		state = (ReadyState)(state | STATE_REQ_ERROR);
	}
	if(controlCommand1Msg.direction == 3
		or controlCommand1Msg.max_speed > 24
		or not controlCommand1Msg.speed_mode
		or controlCommand1Msg.torque_limit != 100
		or controlCommand1Msg.park_brake_off //??? TODO check this
		or controlCommand1Msg.throttle_pot != 0)
	{
		state = (ReadyState)(state | CTRL_CMD1_ERROR);
	}
	//DONT DETELE THESE WILL BE USED LATER
	//	if(controlCommand1Msg.direction == 3)
	//	{
	//		state = (ReadyState)(state | CTRL_CMD1_DIR_ERROR);
	//	}
	//	if(controlCommand1Msg.max_speed > 24)
	//	{
	//		state = (ReadyState)(state | CTRL_CMD1_MAX_SPEED_ERROR);
	//	}
	//	if(not controlCommand1Msg.speed_mode)
	//	{
	//		state = (ReadyState)(state | CTRL_CMD1_SPEED_MODE_ERROR);
	//	}
	//	if(controlCommand1Msg.torque_limit != 100)
	//	{
	//		state = (ReadyState)(state | CTRL_CMD1_TORQUE_LIMIT_ERROR);
	//	}
	//	if(controlCommand1Msg.park_brake_off)
	//	{
	//		state = (ReadyState)(state | CTRL_CMD1_PARK_BRAKE_ERROR);
	//	}
	//	if(controlCommand1Msg.throttle_pot != 0)
	//	{
	//		state = (ReadyState)(state | CTRL_CMD1_THROTTLE_POT_ERROR);
	//	}
	if(state == 0x521)
	{
		state = AUTONOMOUS_VALID;
	}
	return state;
}

void SessionControlNode::MakeAutonomousValid(const ReadyState & reason)
{
	if((reason & AUTONOMY_READY) == 0)
	{
		MakeAutonomyReady(reason);
	}
	else if((reason & STATE_REQ_ERROR) != 0)
	{
		podStateRequest = AUTONOMOUS;
	}
	else if((reason & CTRL_CMD1_ERROR) != 0)
	{
		generateUserInstruction(CTRL_CMD1_SET_ERROR, true);
	}
}

void SessionControlNode::MakeSessionCancelled()
{
	podStateRequest = NO_SESSION;
	sessionId = 0;
}

/////////////////////////////////////////////////////////////////////
//REVOKE TRUE if
//AUTONOMOUS --> NO SESSION
//<driver turn steering wheel> OR
//<driver touches brake> OR
//<driver touches throttle> OR
//<driver changes direction gear select from FWD>
/////////////////////////////////////////////////////////////////////
ReadyState SessionControlNode::GetRevokeTrueState()
{
	ReadyState state = REVOKE_TRUE;
	if(epsMsg.manual_steer_angle != 0)
	{
		state = (ReadyState)(state | MANUAL_STEERING_ERROR);
	}
	if(epsMsg.brake_pedal_force > BRAKE_PEDAL_FORCE_LOWER)
	{
		state = (ReadyState)(state | MANUAL_BRAKE_ERROR);
	}
	if(not powertrainMsg.switch_forward)
	{
		state = (ReadyState)(state | MANUAL_GEAR_ERROR);
	}
	if(powertrainMsg.throttle_ch1 > 0 or powertrainMsg.throttle_ch2 > 0)
	{
		state = (ReadyState)(state | MANUAL_THROTTLE_ERROR);
	}
	return state;
}

void SessionControlNode::GetRevokeReason(const ReadyState & reason)
{
	if((reason & MANUAL_STEERING_ERROR) != 0)
	{
		generateUserInstruction(MANUAL_STEERING_APPLIED, true);
	}
	else if((reason & MANUAL_BRAKE_ERROR) != 0)
	{
		generateUserInstruction(MANUAL_BRAKING_APPLIED, true);
	}
	else if((reason & MANUAL_GEAR_ERROR) != 0)
	{
		generateUserInstruction(GEAR_CHANGED, true);
	}
	else if((reason & MANUAL_THROTTLE_ERROR) != 0)
	{
		generateUserInstruction(MANUAL_THROTTLE_APPLIED, true);
	}
}

/////////////////////////////////////////////////////////////////////
//OVERRIDE TRUE if
//NO SESSION or NEW SESSION or STEERING CENTRING or PRIMED or AUTONOMOUS --> OVERRIDE
//POD NOT READY OR
//ACS_HANDSHAKE <seed/key failed> OR
//(if fitted) LIDAR_HANDSHAKE <seed/key failed> OR
//ACS_CTRL_CMD1:FRAME_CNT <failed> OR
//<CAN NOT OK> OR
//[Session ID is inconsistent OR
//STATE_REQ = NO_SESSION] AND
//[vehicle speed >0 OR <park brake = OFF]]
/////////////////////////////////////////////////////////////////////
ReadyState SessionControlNode::GetOverrideTrueState()
{
	ReadyState state = OVERRIDE_TRUE;
	state = (ReadyState)(state | GetPodReadyState() | GetStationaryState() | GetHandshakeGoodState());
	if(podStateRequest == NO_SESSION)
	{
		state = (ReadyState)(state | STATE_REQ_NO_SESSION_ERROR);
	}
	if(powertrainMsg.vehicle_speed > 0)
	{
		state = (ReadyState)(state | VEHICLE_SPEED_ERROR);
	}
	if(not parkBrakeMsg.park_brake)
	{
		state = (ReadyState)(state | PARK_BRAKE_OFF_ERROR);
	}
	return state;
}

void SessionControlNode::GetOverrideTrueReason(const ReadyState & reason)
{
	if((reason & POD_READY) == 0)
	{
		generateUserInstruction(POD_NOT_READY, false);
	}
	else if((reason & HANDSHAKE_GOOD) == 0)
	{
		generateUserInstruction(HANDSHAKE_NOT_GOOD, false);
	}
	else if((reason & STATE_REQ_NO_SESSION_ERROR) != 0)
	{
		ROS_INFO("NO_SESSION requested.");
	}
	else if((reason & VEHICLE_SPEED_ERROR) != 0)
	{
		generateUserInstruction(POD_SPEED_NOT_0, true);
	}
	else if((reason & PARK_BRAKE_OFF_ERROR) != 0)
	{
		generateUserInstruction(PARK_BRAKE_NOT_ON, true);
	}
}

/////////////////////////////////////////////////////////////////////
//OVERRIDE FALSE if
//OVERRIDE --> NO SESSION
//[STATIONARY OR
//[<driver turn steering wheel> OR
//<driver touches brake> OR
//<driver touches accelerator>]]
//AND
//POD READY
/////////////////////////////////////////////////////////////////////
void SessionControlNode::MakeOverrideFalse(const ReadyState & reason)
{
	if((reason & POD_READY) == 0)
	{
		MakePodReady(reason);
	}
	else if((reason & POD_STATIONARY) == 0)
	{
		MakePodStationary(reason);
	}
	else if((reason & HANDSHAKE_GOOD) == 0)
	{
		MakeHandshakeGood(reason);
	}
}

/////////////////////////////////////////////////////////////////////
//ERROR TRUE if
//MANUAL --> ERROR
//POD NOT READY
/////////////////////////////////////////////////////////////////////
ReadyState SessionControlNode::GetErrorTrueState()
{
	ReadyState state = ERROR_TRUE;
	state = (ReadyState)(state| GetPodReadyState());
	return state;
}

/////////////////////////////////////////////////////////////////////
//ERROR FALSE if
//ERROR --> MANUAL
//STATIONARY AND
//POD READY
/////////////////////////////////////////////////////////////////////
void SessionControlNode::MakeErrorFalse(const ReadyState & reason)
{
	if((reason & POD_READY) == 0)
	{
		MakePodReady(reason);
	}
}

/////////////////////////////////////////////////////////////////////
//E-STOP TRUE if
//Any State --> E‑STOP
//<E-Stop pressed> OR
//<Front bump sensor pressed> OR
//<Rear bump sensor pressed> OR
//<Internal POD system error> OR
//<POD CAN NOT OK>
/////////////////////////////////////////////////////////////////////
ReadyState SessionControlNode::GetEstopState()
{
	ReadyState state = E_STOP_TRUE;
	if(batteryMsg.estop)
	{
		state = (ReadyState)(state | E_STOP_BUTTON_ERROR);
	}
	if(frontBumper)
		{
			state = (ReadyState)(state | FRONT_BUMPER_ERROR);
	}
	if(rearBumper)
	{
		state = (ReadyState)(state | REAR_BUMPER_ERROR);
	}
	return state;
}

void SessionControlNode::GetEstopReason(const ReadyState & reason)
{
	if((reason & E_STOP_BUTTON_ERROR) != 0)
	{
		generateUserInstruction(E_STOP_PRESSED, true);
	}
	else if((reason & FRONT_BUMPER_ERROR) != 0)
	{
		generateUserInstruction(FRONT_BUMPER_PRESSED, true);
	}
	else if((reason & REAR_BUMPER_ERROR) != 0)
	{
		generateUserInstruction(REAR_BUMPER_PRESSED, true);
	}
}

void SessionControlNode::MakeEstopFalse(const ReadyState & reason)
{
	if((reason & E_STOP_TRUE) == 0)
	{
		generateUserInstruction(CYCLE_KEY, true);
	}
}

/////////////////////////////////////////////////////////////////////
//POD READY if
//<Driver seat occupied and belted> AND
//<Passenger seat belted if occupied> AND
//<Doors are closed> AND
//<Charge door is closed>
/////////////////////////////////////////////////////////////////////
ReadyState SessionControlNode::GetPodReadyState()
{
	ReadyState state = ZERO_STATE;
	if(statusMsg.driver_seat != OCCUPIED_BELTED)
	{
		state = (ReadyState)(state | DRIVER_SEAT_ERROR);
	}
	if(statusMsg.passenger_seat == OCCUPIED)
	{
		state = (ReadyState)(state | PASSENGER_SEAT_ERROR);
	}
	if(statusMsg.door_open)
	{
		state = (ReadyState)(state | POD_DOOR_ERROR);
	}
	if(statusMsg.charge_door_open)
	{
		state = (ReadyState)(state | CHARGE_DOOR_ERROR);
	}
	if(state == ZERO_STATE)
	{
		state = POD_READY;
	}
	return state;
}

void SessionControlNode::MakePodReady(const ReadyState & reason)
{
	if((reason & CHARGE_DOOR_ERROR) != 0)
	{
		generateUserInstruction(CHARGE_DOOR, true);
	}
	else if((reason & PASSENGER_SEAT_ERROR) != 0)
	{
		generateUserInstruction(PASSENGER_SEAT, true);
	}
	else if((reason & POD_DOOR_ERROR) != 0)
	{
		generateUserInstruction(POD_DOOR, true);
	}
	else if((reason & DRIVER_SEAT_ERROR) !=0)
	{
		generateUserInstruction(DRIVER_SEAT, true);
	}
}

/////////////////////////////////////////////////////////////////////
//STATIONARY if
//<vehicle stationary> AND
//<parking brake ON>
/////////////////////////////////////////////////////////////////////
ReadyState SessionControlNode::GetStationaryState()
{
	ReadyState state = ZERO_STATE;
	if(powertrainMsg.vehicle_speed != 0)
	{
		state = (ReadyState)(state | POD_STATIONARY_ERROR);
	}
	if(not parkBrakeMsg.park_brake)
	{
		state = (ReadyState)(state | PARK_BRAKE_ON_ERROR);
	}
	if(state == ZERO_STATE)
	{
		state = POD_STATIONARY;
	}
	return state;
}

void SessionControlNode::MakePodStationary(const ReadyState & reason)
{
	if((reason & POD_STATIONARY_ERROR) != 0)
	{
		generateUserInstruction(POD_SPEED_NOT_0, true);
	}
	else if((reason & PARK_BRAKE_ON_ERROR) != 0)
	{
		generateUserInstruction(PARK_BRAKE_NOT_ON, true);
	}
}

/////////////////////////////////////////////////////////////////////
//HANDSHAKE GOOD if
//ACS_CTRL_CMD1:SESSION_ID > 0 AND
//ACS_CTRL_ CMD1:FRAME_CNT <running> AND
//ACS_HANDSHAKE <seed/key running> AND
//ACS_HANDSHAKE:SESSION_ID > 0 AND
//(if fitted) LIDAR_HANDSHAKE <seed/key running>
/////////////////////////////////////////////////////////////////////
ReadyState SessionControlNode::GetHandshakeGoodState()
{
	ReadyState state = ZERO_STATE;
	if(handshakeMsg.qos < 15)
	{
		state = (ReadyState)(state | QOS_ERROR);
	}
	if(state == ZERO_STATE)
	{
		state = HANDSHAKE_GOOD;
	}
	return state;
}

void SessionControlNode::MakeHandshakeGood(const ReadyState & reason)
{
	if((reason & QOS_ERROR) != 0)
	{
		generateUserInstruction(HANDSHAKE_NOT_GOOD, true);
	}
}

/////////////////////////////////////////////////////////////////////
//AUTONOMY READY if
//HANDSHAKE GOOD AND
//<user releases brake> AND  --!!!!!this is wrong .
//STATIONARY AND
//<Gear in FWD> AND
//<No throttle> AND
//POD READY
/////////////////////////////////////////////////////////////////////
ReadyState SessionControlNode::GetAutonomyReadyState()
{
	ReadyState state = ZERO_STATE;
	state = (ReadyState)(state | GetPodReadyState() | GetStationaryState() | GetHandshakeGoodState());

	if(not powertrainMsg.switch_forward)
	{
		state = (ReadyState)(state | GEAR_FWD_ERROR);
	}
	if(powertrainMsg.throttle_ch1 != 0 or powertrainMsg.throttle_ch2 != 0)
	{
		state = (ReadyState)(state | THROTTLE_ERROR);
	}
	if(state == 0x121)
	{
		state = (ReadyState)(state |AUTONOMY_READY);
	}
	return state;
}

void SessionControlNode::MakeAutonomyReady(const ReadyState & reason)
{
	if((reason & POD_READY) == 0)
	{
		MakePodReady(reason);
	}
	else if((reason & POD_STATIONARY) == 0)
	{
		MakePodStationary(reason);
	}
	else if((reason & HANDSHAKE_GOOD) == 0)
	{
		MakeHandshakeGood(reason);
	}
	else if((reason & GEAR_FWD_ERROR) != 0)
	{
		generateUserInstruction(GEAR_SWITCH_NOT_FWD, true);
	}
	else if((reason & THROTTLE_ERROR) != 0)
	{
		generateUserInstruction(THROTTLE_NOT_0, true);
	}
}

}
}
