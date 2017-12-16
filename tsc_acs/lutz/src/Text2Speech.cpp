/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/Text2SpeechNode.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "text2speech");

	tsc_acs::lutz::Text2SpeechNode node;
	node.spin();

	return 0;
}


