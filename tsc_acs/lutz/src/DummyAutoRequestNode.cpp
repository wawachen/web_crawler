/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "ros/ros.h"
#include "lutz/AutonomousRequest.h"
#include "can_msgs/Frame.h"


namespace tsc_acs {
namespace lutz {

class DummyAutoRequestNode
{
public:
	DummyAutoRequestNode()
	{
//		sub = node.subscribe("pod_can_rx", 10, &DummyAutoRequestNode::handleAutoRequest, this);

//		handleAutoRequest();
		pub = node.advertise< ::lutz::AutonomousRequest>("autonomous_request", 10, true);
	}
	void spin()
	{
		handleAutoRequest();
		ros::spin();
	}
private:
//	void handleAutoRequest(const can_msgs::Frame& msg)
	void handleAutoRequest()
	{
		::lutz::AutonomousRequest msgOut;
		msgOut.header.frame_id = "";
		msgOut.header.seq = 0;
		msgOut.header.stamp = ros::Time::now();

		msgOut.autoRequest = true;
		pub.publish(msgOut);
	}

private:
	ros::NodeHandle node;
	ros::Publisher pub;
	ros::Subscriber sub;
};

}}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_request");

    tsc_acs::lutz::DummyAutoRequestNode node;
    node.spin();

    return 0;
}

