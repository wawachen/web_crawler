/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "mobileye/MobileyeNode.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace mobileye {

enum MobileyeCanId
{
    Details_Lane                    = 0x737,
    Details_ObstacleDataA           = 0x739,
    Details_ObstacleDataA_2ndObs    = 0x73C,
    Details_ObstacleDataA_3ndObs    = 0x73F,
    Details_ObstacleDataA_4thObs    = 0x742,
    Details_ObstacleDataB           = 0x73A,
    Details_ObstacleDataB_2ndObs    = 0x73D,
    Details_ObstacleDataB_3ndObs    = 0x740,
    Details_ObstacleDataB_4thObs    = 0x743,
    Details_ObstacleDataC           = 0x73B,
    Details_ObstacleDataC_2ndObs    = 0x73E,
    Details_ObstacleDataC_3ndObs    = 0x741,
    Details_ObstacleDataC_4thObs    = 0x744,
    Details_ObstacleStatus          = 0x738,
    Fixed_FOE_signals               = 0x650
};

MobileyeNode::MobileyeNode()
{
	ReportVersion(node);
  sub = node.subscribe("mobileye_can_rx", 1000, &MobileyeNode::rxCallback, this);

  pubObstacleStatus = node.advertise< ::mobileye::ObstacleStatus>("mobileye/obstacle_status", 100);
}

void MobileyeNode::spin()
{
  ros::spin();
}

void MobileyeNode::rxCallback(const can_msgs::Frame& msg)
{
  switch (msg.id)
  {
    case Details_ObstacleStatus:
    {
      handleObstacleStatus(msg);
    }
    break;
    case Details_Lane:
    default:
    {
      ROS_INFO("Unhandled CAN message: %x", msg.id);
    }
    break;
  }
}

}
}
