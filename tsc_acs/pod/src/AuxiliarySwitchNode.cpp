/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "pod/AuxiliarySwitchNode.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace lutz {

AuxiliarySwitchNode::AuxiliarySwitchNode(): sendRate(2)
{
	ReportVersion(node);
	subAcsAuxDemand = node.subscribe("pod/acs_auxiliary_demand", 10, &AuxiliarySwitchNode::handleXnavAuto, this);
	subXboxAuxDemand = node.subscribe("pod/xbox_auxiliary_demand", 10, &AuxiliarySwitchNode::handleXboxJoy, this);
	subTurnigyAuxDemand = node.subscribe("/pod/turnigy_auxiliary_demand", 10, &AuxiliarySwitchNode::handleMavJoy, this);

	pub = node.advertise< ::pod::AuxiliaryDemand >("pod/auxiliary_demand", 10, true);
}

void AuxiliarySwitchNode::handleXnavAuto(const ::pod::AuxiliaryDemand & msg)
{
	acsAuxDemand = msg;
}

void AuxiliarySwitchNode::handleXboxJoy(const ::pod::AuxiliaryDemand & msg)
{
	xboxAuxDemand = msg;
}
void AuxiliarySwitchNode::handleMavJoy(const ::pod::AuxiliaryDemand & msg)
{
	turnigyAuxDemand = msg;
}

void AuxiliarySwitchNode::spin()
{
	while (ros::ok())
	{
		handlePublisher();
		ros::spinOnce();
		sendRate.sleep();
	}
}

void AuxiliarySwitchNode::handlePublisher()
{

	auxiliaryDemand.indicateLeft = turnigyAuxDemand.indicateLeft ||
                                  xboxAuxDemand.indicateLeft ||
                                  acsAuxDemand.indicateLeft;
	auxiliaryDemand.indicateRight = turnigyAuxDemand.indicateRight ||
                                  xboxAuxDemand.indicateRight ||
                                  acsAuxDemand.indicateRight;
	auxiliaryDemand.hornOn = turnigyAuxDemand.hornOn ||
                                  xboxAuxDemand.hornOn ||
                                  acsAuxDemand.hornOn;

	pub.publish(auxiliaryDemand);
}

}}
