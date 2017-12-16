/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include "lutz/PodHeader.h"

namespace tsc_acs {
namespace lutz {

//! ROS Node for listening to LUTZ Pod CAN data.

//! This node listens for CAN frames from the pod published on the
//! pod_can_rx topic.  It interprets these into an appropriate lutz message
//! and publishes it on one of many topics.
class PodRxNode
{
public:
  //! Constructor

  //! Subscribes to the pod_can_rx topic and advertises the many pod specific
  //! topics.
  PodRxNode();

  //! Executes node activity
  void spin();

private:
  static bool ShouldPublish(const can_msgs::Frame & msg, uint64_t stateMask,
                            uint64_t & state);
  void rxCallback(const can_msgs::Frame & msg);

  static void ReadPodHeader(::lutz::PodHeader & header, const can_msgs::Frame & msg);
  static void handlePOD_SW_VER(const can_msgs::Frame & msg, uint64_t & currentState,
                        ros::Publisher & pub);
  static void handlePOD_ULTRASONIC_STATUS(const can_msgs::Frame & msg,
                                   uint64_t & currentState,
                                   ros::Publisher & pub);
  static void handlePOD_DEBUG_COMMON(const can_msgs::Frame & msg,
                                   uint64_t & currentState,
                                   ros::Publisher & pub);
  static void handlePOD_EPS_STATUS(const can_msgs::Frame & msg,
                                              uint64_t & currentState,
                                              ros::Publisher & pub);

  void handlePOD_AUX_POWER_STATUS(const can_msgs::Frame & msg);
  void handlePOD_BATTERY_STATUS(const can_msgs::Frame & msg);
  void handlePOD_DEBUG(const can_msgs::Frame & msg);
  void handlePOD_DEBUG2(const can_msgs::Frame & msg);
  void handlePOD_DI_STATE(const can_msgs::Frame & msg);
  void handlePOD_EPB_STATUS(const can_msgs::Frame & msg);
  void handlePOD_EPB_SW_VER(const can_msgs::Frame & msg);
  void handlePOD_EPI_SW_VER(const can_msgs::Frame & msg);
  void handlePOD_EPO_SW_VER(const can_msgs::Frame & msg);
  void handlePOD_EPS_INBOARD_STATUS(const can_msgs::Frame & msg);
  void handlePOD_EPS_OUTBOARD_STATUS(const can_msgs::Frame & msg);
  void handlePOD_EPS_STATUS(const can_msgs::Frame & msg);
  void handlePOD_FAULT_CODES(const can_msgs::Frame & msg);
  void handlePOD_FRONT_ULTRASONIC_STATUS(const can_msgs::Frame & msg);
  void handlePOD_GWY_SW_VER(const can_msgs::Frame & msg);
  void handlePOD_HANDSHAKE(const can_msgs::Frame & msg);
  void handlePOD_IMU1_STATUS(const can_msgs::Frame & msg);
  void handlePOD_IMU2_STATUS(const can_msgs::Frame & msg);
  void handlePOD_POWERTRAIN_STATUS(const can_msgs::Frame & msg);
  void handlePOD_REAR_ULTRASONIC_STATUS(const can_msgs::Frame & msg);
  void handlePOD_STATUS(const can_msgs::Frame & msg);
  void handlePOD_VMS_SW_VER(const can_msgs::Frame & msg);
  void handlePOD_WDG_SW_VER(const can_msgs::Frame & msg);
  void handlePOD_WHL_CNT(const can_msgs::Frame & msg);
  void handlePOD_WHL_SW_VER(const can_msgs::Frame & msg);

private:
  ros::NodeHandle node;
  ros::Subscriber sub;

  ros::Publisher pubPOD_AUX_POWER_STATUS;
  ros::Publisher pubPOD_BATTERY_STATUS;
  ros::Publisher pubPOD_DEBUG;
  ros::Publisher pubPOD_DEBUG2;
  ros::Publisher pubPOD_DI_STATE;
  ros::Publisher pubPOD_EPB_STATUS;
  ros::Publisher pubPOD_EPB_SW_VER;
  ros::Publisher pubPOD_EPI_SW_VER;
  ros::Publisher pubPOD_EPO_SW_VER;
  ros::Publisher pubPOD_EPS_INBOARD_STATUS;
  ros::Publisher pubPOD_EPS_OUTBOARD_STATUS;
  ros::Publisher pubPOD_EPS_STATUS;
  ros::Publisher pubPOD_FAULT_CODES;
  ros::Publisher pubPOD_FRONT_ULTRASONIC_STATUS;
  ros::Publisher pubPOD_GWY_SW_VER;
  ros::Publisher pubPOD_HANDSHAKE;
  ros::Publisher pubPOD_IMU1_STATUS;
  ros::Publisher pubPOD_IMU2_STATUS;
  ros::Publisher pubPOD_POWERTRAIN_STATUS;
  ros::Publisher pubPOD_REAR_ULTRASONIC_STATUS;
  ros::Publisher pubPOD_STATUS;
  ros::Publisher pubPOD_VMS_SW_VER;
  ros::Publisher pubPOD_WDG_SW_VER;
  ros::Publisher pubPOD_WHL_CNT;
  ros::Publisher pubPOD_WHL_SW_VER;

  ros::Publisher pubPOD_HEADER;

  uint64_t statePOD_AUX_POWER_STATUS;
  uint64_t statePOD_BATTERY_STATUS;
  uint64_t statePOD_DEBUG;
  uint64_t statePOD_DEBUG2;
  uint64_t statePOD_DI_STATE;
  uint64_t statePOD_EPB_STATUS;
  uint64_t statePOD_EPB_SW_VER;
  uint64_t statePOD_EPI_SW_VER;
  uint64_t statePOD_EPO_SW_VER;
  uint64_t statePOD_EPS_INBOARD_STATUS;
  uint64_t statePOD_EPS_OUTBOARD_STATUS;
  uint64_t statePOD_EPS_STATUS;
  uint64_t statePOD_FAULT_CODES;
  uint64_t statePOD_FRONT_ULTRASONIC_STATUS;
  uint64_t statePOD_GWY_SW_VER;
  uint64_t statePOD_HANDSHAKE;
  uint64_t statePOD_IMU1_STATUS;
  uint64_t statePOD_IMU2_STATUS;
  uint64_t statePOD_POWERTRAIN_STATUS;
  uint64_t statePOD_REAR_ULTRASONIC_STATUS;
  uint64_t statePOD_STATUS;
  uint64_t statePOD_VMS_SW_VER;
  uint64_t statePOD_WDG_SW_VER;
  uint64_t statePOD_WHL_CNT;
  uint64_t statePOD_WHL_SW_VER;


};

}
}
