/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include <std_msgs/String.h>

namespace tsc_acs {
namespace lutz {

//! ROS Node for turning taking keyboard entry and publishing it.
class KeyEntryNode
{
public:
	//! Constructor
	KeyEntryNode();

	//! Executes node activity
	void spin();

private:
	void handlePublisher();

private:
	ros::NodeHandle node;
	ros::Publisher pub;
};
}}
