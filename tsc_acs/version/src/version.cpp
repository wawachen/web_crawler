/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "../version.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bacs_version");
	ros::NodeHandle node;
	tsc_acs::ReportVersion(node);

	ros::spin();
}
