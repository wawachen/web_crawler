/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/
#pragma once
#include "ros/ros.h"
#include "bacs_version/Version.h"

namespace tsc_acs {

	#include "git_info.h"

	const char * VERSION_STRING = (GIT_DESC " " GIT_BRANCH);
	const int VERSION_MAJOR = GIT_MAJOR_VERSION;
	const int VERSION_MINOR = GIT_MINOR_VERSION;
	const int VERSION_BUILD = GIT_BUILD_VERSION;
	const bool VERSION_VALID = (VERSION_MAJOR>=0 && GIT_LOCAL_CHANGES == 0 && GIT_ADDITIONAL_COMMITS == 0);

	void ReportVersion(ros::NodeHandle & node)
	{
		ROS_INFO("Node: %s", ros::this_node::getName().c_str());
		ROS_INFO("Version: %s", VERSION_STRING);
		ROS_INFO("Version: %d.%d.%d", VERSION_MAJOR, VERSION_MINOR, VERSION_BUILD);
		if (!VERSION_VALID)
		{
			ROS_WARN("VERSION NUMBER IS NOT VALID");
		}
		
		static ros::Publisher pub = node.advertise< ::bacs_version::Version>("/bacs_version", 3, true);
			// Made static so that desctructor does not get called.  Without static the message is not published.

		::bacs_version::Version msg;
		msg.node = ros::this_node::getName().c_str();
		msg.major_version = VERSION_MAJOR;
		msg.minor_version = VERSION_MINOR;
		msg.build_version = VERSION_BUILD;
		msg.valid = VERSION_VALID;
		msg.versionString = VERSION_STRING;
		msg.localChanges = GIT_LOCAL_CHANGES;
		msg.additionalCommits = GIT_ADDITIONAL_COMMITS;

		pub.publish(msg);
	}

}