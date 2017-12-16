/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodTxNode.h"
#include "can_msgs/Frame.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace lutz {

PodTxNode::PodTxNode()
  : frameCount(0), sessionId(0)
{
	ReportVersion(node);
  subACS_AUX_POWER_REQ = node.subscribe("aux_power_request", 10, &PodTxNode::transmitACS_AUX_POWER_REQ, this);
  subACS_CTRL_CMD1 = node.subscribe("control_command1", 10, &PodTxNode::transmitACS_CTRL_CMD1, this);
  subACS_CTRL_CMD2 = node.subscribe("control_command2", 10, &PodTxNode::transmitACS_CTRL_CMD2, this);
  subACS_HANDSHAKE = node.subscribe("acs_handshake", 10, &PodTxNode::transmitACS_HANDSHAKE, this);

  subSessionControl = node.subscribe("session_control", 10, &PodTxNode::rxSessionControl, this);

  pub = node.advertise<can_msgs::Frame>("pod_can_tx", 10, true);
}

void PodTxNode::rxSessionControl(const ::lutz::SessionControl & msg)
{
  sessionId = msg.session_id;
}

void PodTxNode::spin()
{
  ros::spin();
}

}
}




