/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "ibeo/FusionNode.h"
#include "ibeo/FusionObjectList.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace ibeo {

enum DataType
{
  FusionScanData = 0x2205,
  FusionObjectData = 0x2280,
  FusionImage = 0x2403,
  FusionVehicleState = 0x2807
};

FusionNode::FusionNode(bool reportUnprocessedMessages)
: reportUnprocessedMessages(reportUnprocessedMessages)
{
	ReportVersion(node);
  sub = node.subscribe("ibeo/message", 1000, &FusionNode::rxCallback, this);

  pubFusionScan = node.advertise< ::ibeo::FusionScan>("ibeo/fusion_scan", 10);
  pubFusionPointCloud = node.advertise<sensor_msgs::PointCloud2>("ibeo/point_cloud", 10);
  pubFusionObject = node.advertise< ::ibeo::FusionObjectList>("ibeo/fusion_object", 10);
}

void FusionNode::spin()
{
  ros::spin();
}

void FusionNode::rxCallback(const ::ibeo::Message& msg)
{
  switch(msg.ibeo_header.data_type)
  {
    case FusionScanData:
      if (pubFusionScan.getNumSubscribers() > 0)
        handleFusionScan(msg);
      if (pubFusionPointCloud.getNumSubscribers() > 0)
        handleFusionScan_PointCloud(msg);
      break;
    case FusionObjectData:
      if (pubFusionObject.getNumSubscribers() > 0)
        handleFusionObject(msg);
    default:
      if (reportUnprocessedMessages)
        ROS_INFO("IBEO  : %04X : %d",
                 msg.ibeo_header.data_type, msg.ibeo_header.data_size);
      break;
  }
}

}
}
