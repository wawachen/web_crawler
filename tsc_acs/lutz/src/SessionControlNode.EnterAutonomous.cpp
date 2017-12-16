/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/SessionControlNode.h"

namespace tsc_acs {
namespace lutz {

void SessionControlNode::EnterAutonomous()
{
	switch (statusMsg.pod_state)
	{
		case INVALID:
		{
			generateUserInstruction(INVALID_STATE, true);
		}
		break;
		case KEY_OFF:
		{
			MakeKeyManual();
		}
		break;
		case MANUAL:
		{
			MakeKeyAuto();
		}
		break;
		case ERROR:
		{
			tempReadyState = GetErrorTrueState();
			MakeErrorFalse(tempReadyState);
		}
		break;
		case E_STOP:
		{
			tempReadyState = GetEstopState();
			ROS_INFO("E_STOP reason: ");
			GetEstopReason(tempReadyState); //display E_Stop reason
			MakeEstopFalse(tempReadyState); //call E_Stop false to get out of E_Stop state.
		}
		break;
		case NO_SESSION:
		{
			MakeSession0();
		}
		break;
		case NEW_SESSION:
		{
			tempReadyState = GetSessionValidState();
			if((tempReadyState & SESSION_VALID) == 0)
			{
				MakeSessionValid(tempReadyState);
			}
		}
		break;
		case REQ_CENTRE:
		{
			tempReadyState = GetPrimedValidState();
			if((tempReadyState & PRIMED_VALID) == 0)
			{
				MakePrimedValid(tempReadyState);
			}
		}
		break;
		case CHECK_CENTER:
		{
			tempReadyState = GetPrimedValidState();
			if((tempReadyState & PRIMED_VALID) == 0)
			{
				MakePrimedValid(tempReadyState);
			}
		}
		break;
		case PRIMED:
		{
			tempReadyState = GetAutonomousValidState();
			if((tempReadyState & AUTONOMOUS_VALID) == 0)
			{
				MakeAutonomousValid(tempReadyState);
			}
			else
			{
				generateUserInstruction(RELEASE_BRAKE, true);
			}
		}
		break;
		case AUTO_DELAY:
		{
		}
		break;
		case AUTONOMOUS:
		{
		}
		break;
		case OVERRIDE:
		{
			ROS_INFO("OVERRIDE reason: ");
			tempReadyState = GetOverrideTrueState();
			GetOverrideTrueReason(tempReadyState);
			MakeOverrideFalse(tempReadyState);
		}
		break;
		case CHARGING:
		{
			generateUserInstruction(CHARGING_OFF, true);
		}
		break;
		default:
		{
		  ROS_INFO("State : [%d] : NOT DEFINED", statusMsg.pod_state);
		}
		break;
	}
}

}
}
