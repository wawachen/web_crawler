/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "dead_mans_handle/deadMansHandleNode.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dead_mans_handle");
	ros::NodeHandle nodeParam("~");

	float rangeMin = 450;
	nodeParam.getParam("rangeMin", rangeMin);

	float rangeMax = 900;
	nodeParam.getParam("rangeMax", rangeMax);


	tsc_acs::deadMansHandle::deadMansHandleNode node(rangeMin, rangeMax);

	node.spin();

	return 0;
}



