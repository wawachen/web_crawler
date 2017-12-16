/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

namespace tsc_acs {
namespace lutz {

enum UserInstructionsCode
{
	DRIVER_SEAT,				// 0
	PASSENGER_SEAT,				// 1
	POD_DOOR,					// 2
	CHARGE_DOOR,				// 3
	BPL_LOW,					// 4
	GEAR_SWITCH_NOT_FWD,		// 5
	POD_IS_READY,				// 6
	POD_NOT_READY,				// 7
	POD_IS_STATIONARY,			// 8
	POD_NOT_STATIONARY,			// 9
	POD_SPEED_NOT_0,			// 10
	PARK_BRAKE_NOT_ON,			// 11
	HANDSHAKE_IS_GOOD,			// 12
	HANDSHAKE_NOT_GOOD,			// 13
	THROTTLE_NOT_0,				// 14
	AUTONOMY_IS_READY,			// 15
	AUTONOMY_NOT_READY,			// 16
	KEY_NOT_IN_AUTO,			// 17
	CENTRE_STEERING,			// 18
	MANUAL_STEERING_APPLIED,	// 19
    MANUAL_BRAKING_APPLIED,		// 20
	GEAR_CHANGED,				// 21
	MANUAL_THROTTLE_APPLIED,	// 22
	CTRL_CMD1_SET_ERROR,		// 23
	RELEASE_BRAKE,				// 24
	E_STOP_PRESSED,				// 25
	FRONT_BUMPER_PRESSED,		// 26
	REAR_BUMPER_PRESSED,		// 27
	CYCLE_KEY,					// 28
	KEY_ON,						// 29
	CHARGING_OFF,				// 30
	DRIVE_BY_WIRE_REQ,			// 31
	DRIVE_BY_WIRE_CANCEL,		// 32
	INVALID_STATE,				// 33

	UI_CODE_COUNT,

	NO_UI_CODE = 255
};

//extern const int InstructionsCode[] = {0, 1, 2, 3, 4, 5, 10, 11, 17, 18, 24, 28, 29, 30};

const char *const Instructions[] =
{
	"Occupy driver's seat and fasten seat belt.", // 0
	"Either remove passenger or fasten the seat belt.", // 1
	"Close both pod doors.", // 2
	"Close the charge door.", // 3
	"Apply brake.", // 4
	"Put the gear in forward position.", // 5
	"Pod is ready.", // 6
	"Pod is not ready.", // 7
	"Pod is stationary.", // 8
	"Pod is not stationary.", // 9
	"Pod speed is not zero. Stop the pod.", // 10
	"Park brake is not on. stop the pod and wait a few seconds. Cycle the gears if needed.", // 11
	"Handshake is good.", // 12
	"Handshake failed. Check Handshake", // 13
	"Throttle is not 0.", // 14
	"Ready for Autonomy.", // 15
	"Not ready for Autonomy.", // 16
	"Turn the key to Auto.", // 17
	"Centre the steering wheel.", // 18
	"Manual steering was applied..", // 19
	"Manual braking was applied.", // 20
	"gear was changed manually.", // 21
	"Manual throttle was applied.", // 22
	"Control command 1 not set properly.", // 23
	"Release bake.", // 24
	"E_STOP has been pressed.", // 25
	"Front Bump strip has been activated.", // 26
	"Rear Bump strip has been activated.", // 27
	"Cycle the keys to get out of E_STOP, or reset the E_Stop button.", // 28
	"Switch the key on.", // 29
	"Unplug charging cable and restart the pod.", // 30
	"Drive by wire requested.", // 31
	"Drive by wire request cancelled.", // 32
	"Pod state is INVALID, check power and restart the pod.", // 33
};

}
}
