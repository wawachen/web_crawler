/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "ros/ros.h"
#include <std_msgs/String.h>

namespace tsc_acs {
namespace lutz {

class KeyEntryNode
{
public:
	KeyEntryNode()
	{
		pub = node.advertise< std_msgs::String>("keyboard_input", 10, true);
	}

	void spin()
	{
		while (ros::ok())
		{
			handlePublisher();
			ros::spinOnce();
		}
	}

private:
	void handlePublisher()
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

private:
	ros::NodeHandle node;
	ros::Publisher pub;
};

}}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_node");

    tsc_acs::lutz::KeyEntryNode node;
    node.spin();

    return 0;
}
