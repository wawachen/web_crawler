/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "ibeo/FusionScan.h"
#include "ibeo/Message.h"
#include "ibeo/Point2D32.h"
#include "sensor_msgs/PointCloud2.h"

namespace tsc_acs {
namespace ibeo {

//! Ibeo Fusion ROS Node

//! A ROS node that subscribes to the ibeo/messages.
//! Incoming messages are interpreted based on their dataType and a
//! domain specific message is published on an appropriate channel.
class FusionNode
{
public:
  FusionNode(bool reportUnprocessedMessages);
  void spin();

private:
  void rxCallback(const ::ibeo::Message& msg);

  void handleFusionScan_PointCloud(const ::ibeo::Message & msg);
  void handleFusionScan(const ::ibeo::Message & msg);
  static void readScannerInfo(::ibeo::ScannerInfo & scannerInfo, const uint8_t * data);
  static void readResolution(::ibeo::Resolution & resolution, const uint8_t * data);
  static void readScanPoint(::ibeo::ScanPoint & scanPoint, const uint8_t * data);

  void handleFusionObject(const ::ibeo::Message & msg);
  static void readPoint2D(::ibeo::Point2D32 & point, const uint8_t * data);
private:
  ros::NodeHandle node;
  ros::Subscriber sub;

  ros::Publisher pubFusionScan;
  ros::Publisher pubFusionPointCloud;
  ros::Publisher pubFusionObject;
  sensor_msgs::PointCloud2 pc2;

  bool reportUnprocessedMessages;
};

}
}
