/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "ros/ros.h"
#include "pod/PodDemandSource.h"
#include "pod/AuxiliaryDemand.h"
#include "can_msgs/Frame.h"
#include <ncurses.h>

namespace tsc_acs {
namespace lutz {

class DummyACSNode
{
private:
  bool abdicate;
  float speed;
  float steer;

  std::string keyCodeString;
public:
	DummyACSNode()
      : abdicate(false), speed(0), steer(0)
	{
	    initscr();
	    cbreak();
	    nodelay(stdscr, TRUE);
	    scrollok(stdscr, TRUE);

		pubPodDemand = node.advertise< ::pod::PodDemandSource>("pod/acs_pod_demand", 10, true);
//		pubAuxDemand = node.advertise< ::pod::AuxiliaryDemand>("pod/acs_auxiliary_demand", 10, true);
	}

	void spin()
	{
	    ros::Rate rate(10);
	    while(ros::ok())
	    {
	      int ch;
	      do
	      {
            ch = getch();
            switch(ch)
            {
              case ERR:
                break;
              case 'a':
              case 'A':
                abdicate = !abdicate;
                break;
              case 'l':
              case 'L':
                steer -= 0.1f;
                break;
              case 'r':
              case 'T':
                steer += 0.1f;
                break;
              case 'u':
              case 'U':
                speed += 1.0f;
                break;
              case 'd':
              case 'D':
                speed -= 1.0f;
                break;
              case 's':
              case 'S':
                speed =0;
                break;
              case 'z':
              case 'Z':
                steer =0;
                break;
              default:
                ROS_INFO("%i : %c\r\n", ch, (char)ch);
                break;
            }
          } while (ch != ERR);

	      publish();
	      ros::spinOnce();

	      rate.sleep();
	    }
	}

private:
	void publish()
	{
		::pod::PodDemandSource msgPodDemandOut;

		msgPodDemandOut.abdicate = abdicate;
        msgPodDemandOut.podDemand.header.stamp = ros::Time::now();
		msgPodDemandOut.podDemand.speed = speed;
		msgPodDemandOut.podDemand.steer = steer;
		msgPodDemandOut.podDemand.driveByWireRequest = true;
		msgPodDemandOut.podDemand.source = ros::this_node::getName();
		pubPodDemand.publish(msgPodDemandOut);

/*
		::pod::AuxiliaryDemand msgAuxDemandOut;
		msgAuxDemandOut.header.stamp = ros::Time::now();

		msgAuxDemandOut.hornOn = 0;
		msgAuxDemandOut.indicateLeft = 0;
		msgAuxDemandOut.indicateRight = 0;
		msgAuxDemandOut.mimicLights = 0;
		pubAuxDemand.publish(msgAuxDemandOut);
*/	}

private:
	ros::NodeHandle node;
	ros::Publisher pubPodDemand;
//	ros::Publisher pubAuxDemand;
};

}}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_acs");

    tsc_acs::lutz::DummyACSNode node;
    node.spin();

    return 0;
}

