/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once
#include <string.h>
#include "ros/ros.h"
//#include <std_msgs/Int16.h>

#include "lutz/UICode.h"
#include "lutz/UserInstructions.h"
#include "lutz/PodStates.h"
#include "sound_play/sound_play.h"

namespace tsc_acs{
namespace lutz{

class Text2SpeechNode
{
public:
	// Constructor
	Text2SpeechNode();

	// Executes node activity
	void spin();

private:
	void sayInstruction(const ::lutz::UICode & msg);

private:
	ros::NodeHandle node;
	ros::Subscriber subInstruction;
	ros::Rate sendRate;

	UserInstructionsCode speechState;
	PodStateId podState;
	sound_play::SoundClient soundClient;
};

}
}
