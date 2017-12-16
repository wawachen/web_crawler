/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/Text2SpeechNode.h"
#include<algorithm>
#include "../../version/version.h"

namespace tsc_acs{
namespace lutz{

Text2SpeechNode::Text2SpeechNode() : sendRate(0.3), speechState(NO_UI_CODE), podState(INVALID)
{
	ReportVersion(node);
	subInstruction = node.subscribe("ui_code", 50, &Text2SpeechNode::sayInstruction, this);
}

void Text2SpeechNode::sayInstruction(const ::lutz::UICode & msg)
{
	if(msg.flag)
	{
		if(speechState != msg.instruction && msg.instruction < UI_CODE_COUNT)
		{
            sendRate.sleep();
			soundClient.say(tsc_acs::lutz::Instructions[msg.instruction]);
			speechState = (UserInstructionsCode)msg.instruction;
		}
	}
}

void Text2SpeechNode::spin()
{
	while (ros::ok())
	{
		ros::spinOnce();
	}
}

}
}
