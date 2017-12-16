/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "pod/KeyEntryNode.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace lutz {

KeyEntryNode::KeyEntryNode()
{
	ReportVersion(node);
	pub = node.advertise< std_msgs::String>("keyboard_input", 10, true);
}

void KeyEntryNode::spin()
{
	while (ros::ok())
	{
		handlePublisher();
		ros::spinOnce();
	}
}

void KeyEntryNode::handlePublisher()
{
	std::string inputString;
	std::cout << "Enter string to record: ";
	std::getline(std::cin, inputString);
	std_msgs::String msg;

	//only publish if the inputstring is not empty.
	if(not inputString.empty())
	{

	  msg.data = inputString;
	  pub.publish(msg);
	}
}
}}
