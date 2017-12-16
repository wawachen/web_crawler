/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "lutz/PodRxNode.h"
#include "lutz/PodCanId.h"
#include "lutz/AuxPower.h"
#include "lutz/Battery.h"
#include "lutz/Debug.h"
#include "lutz/EPS.h"
#include "lutz/FaultCodes.h"
#include "lutz/Handshake.h"
#include "lutz/Imu1.h"
#include "lutz/Imu2.h"
#include "lutz/Indicator.h"
#include "lutz/ParkBrake.h"
#include "lutz/Powertrain.h"
#include "lutz/Status.h"
#include "lutz/Steer.h"
#include "lutz/Ultrasonics.h"
#include "lutz/Version.h"
#include "lutz/WheelCount.h"
#include "lutz/PodCanHeader.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace lutz {

PodRxNode::PodRxNode() :    statePOD_AUX_POWER_STATUS(0xFF),
    statePOD_BATTERY_STATUS(0xFF), statePOD_DEBUG(0xFF), statePOD_DEBUG2(0xFF),
    statePOD_DI_STATE(0xFF), statePOD_EPB_STATUS(0xFF), statePOD_EPB_SW_VER(0xFF),
    statePOD_EPI_SW_VER(0xFF), statePOD_EPO_SW_VER(0xFF), statePOD_EPS_INBOARD_STATUS(0xFF),
    statePOD_EPS_OUTBOARD_STATUS(0xFF), statePOD_EPS_STATUS(0xFF), statePOD_FAULT_CODES(0xFF),
    statePOD_FRONT_ULTRASONIC_STATUS(0xFF), statePOD_GWY_SW_VER(0xFF), statePOD_HANDSHAKE(0xFF),
    statePOD_IMU1_STATUS(0xFF), statePOD_IMU2_STATUS(0xFF), statePOD_POWERTRAIN_STATUS(0xFF),
    statePOD_REAR_ULTRASONIC_STATUS(0xFF), statePOD_STATUS(0xFF), statePOD_VMS_SW_VER(0xFF),
    statePOD_WDG_SW_VER(0xFF), statePOD_WHL_CNT(0xFF), statePOD_WHL_SW_VER(0xFF)
{
	ReportVersion(node);
  sub = node.subscribe("pod_can_rx", 10, &PodRxNode::rxCallback, this);

  pubPOD_AUX_POWER_STATUS = node.advertise< ::lutz::AuxPower>("aux_power", 10, true);
  pubPOD_BATTERY_STATUS = node.advertise< ::lutz::Battery>("battery", 10, true);
  pubPOD_DEBUG = node.advertise< ::lutz::Debug>("debug", 10, true);
  pubPOD_DEBUG2 = node.advertise< ::lutz::Debug>("debug2", 10, true);
  pubPOD_DI_STATE = node.advertise< ::lutz::Indicator>("indicator", 10, true);
  pubPOD_EPB_STATUS = node.advertise< ::lutz::ParkBrake>("epb", 10, true);
  pubPOD_EPB_SW_VER = node.advertise< ::lutz::Version>("epb_ver", 10, true);
  pubPOD_EPI_SW_VER = node.advertise< ::lutz::Version>("epi_ver", 10, true);
  pubPOD_EPO_SW_VER = node.advertise< ::lutz::Version>("epo_ver", 10, true);
  pubPOD_EPS_INBOARD_STATUS = node.advertise< ::lutz::Steer>("eps_inboard", 10, true);
  pubPOD_EPS_OUTBOARD_STATUS = node.advertise< ::lutz::Steer>("eps_outboard", 10, true);
  pubPOD_EPS_STATUS = node.advertise< ::lutz::EPS>("eps", 10, true);
  pubPOD_FAULT_CODES = node.advertise< ::lutz::FaultCodes>("fault_codes", 10, true);
  pubPOD_FRONT_ULTRASONIC_STATUS = node.advertise< ::lutz::Ultrasonics>("front_ultrasonic", 10, true);
  pubPOD_GWY_SW_VER = node.advertise< ::lutz::Version>("gwy_ver", 10, true);
  pubPOD_HANDSHAKE = node.advertise< ::lutz::Handshake>("pod_handshake", 10, true);
  pubPOD_IMU1_STATUS = node.advertise< ::lutz::Imu1>("imu1", 10, true);
  pubPOD_IMU2_STATUS = node.advertise< ::lutz::Imu2>("imu2", 10, true);
  pubPOD_POWERTRAIN_STATUS = node.advertise< ::lutz::Powertrain>("powertrain", 10, true);
  pubPOD_REAR_ULTRASONIC_STATUS = node.advertise< ::lutz::Ultrasonics>("rear_ultrasonic", 10, true);
  pubPOD_STATUS = node.advertise< ::lutz::Status>("status", 10, true);
  pubPOD_VMS_SW_VER = node.advertise< ::lutz::Version>("vms_ver", 10, true);
  pubPOD_WDG_SW_VER = node.advertise< ::lutz::Version>("wdg_ver", 10, true);
  pubPOD_WHL_CNT = node.advertise< ::lutz::WheelCount>("wheel_count", 10, true);
  pubPOD_WHL_SW_VER = node.advertise< ::lutz::Version>("whl_ver", 10, true);
}

// The ShouldPublish function decides whether or not to publish the topic based
// on two situations.
// It can either be set to publish as there is a change in the message content,
// or if there are other nodes subscribing to the topic.
bool PodRxNode::ShouldPublish(const can_msgs::Frame & msg, uint64_t stateMask,
                              uint64_t & state)
{
  uint64_t previousState = state;
  state = *((uint64_t *)msg.data.data());

  return (previousState & stateMask) != (state & stateMask);
}

void PodRxNode::ReadPodHeader(::lutz::PodHeader & header, const can_msgs::Frame & msg)
{
  _header & canHeader = *((_header*)msg.data.data());

  header.id = msg.id;
  header.frame_count = canHeader.frameCount;
  header.session_id = canHeader.sessionId;
}

void PodRxNode::rxCallback(const can_msgs::Frame & msg)
{
  switch (msg.id)
  {
    case POD_AUX_POWER_STATUS:
    {
      handlePOD_AUX_POWER_STATUS(msg);
    }
    break;
    case POD_BATTERY_STATUS:
    {
      handlePOD_BATTERY_STATUS(msg);
    }
    break;
    case POD_DEBUG:
    {
      handlePOD_DEBUG(msg);
    }
    break;
    case POD_DEBUG2:
    {
      handlePOD_DEBUG2(msg);
    }
    break;
    case POD_DI_STATE:
    {
      handlePOD_DI_STATE(msg);
    }
    break;
    case POD_EPB_STATUS:
    {
      handlePOD_EPB_STATUS(msg);
    }
    break;
    case POD_EPB_SW_VER:
    {
      handlePOD_EPB_SW_VER(msg);
    }
    break;
    case POD_EPI_SW_VER:
    {
      handlePOD_EPI_SW_VER(msg);
    }
    break;
    case POD_EPO_SW_VER:
    {
      handlePOD_EPO_SW_VER(msg);
    }
    break;
    case POD_EPS_INBOARD_STATUS:
    {
      handlePOD_EPS_INBOARD_STATUS(msg);
    }
    break;
    case POD_EPS_OUTBOARD_STATUS:
    {
      handlePOD_EPS_OUTBOARD_STATUS(msg);
    }
    break;
    case POD_EPS_STATUS:
    {
      handlePOD_EPS_STATUS(msg);
    }
    break;
    case POD_FAULT_CODES:
    {
      handlePOD_FAULT_CODES(msg);
    }
    break;
    case POD_FRONT_ULTRASONIC_STATUS:
    {
      handlePOD_FRONT_ULTRASONIC_STATUS(msg);
    }
    break;
    case POD_GWY_SW_VER:
    {
      handlePOD_GWY_SW_VER(msg);
    }
    break;
    case POD_HANDSHAKE:
    {
      handlePOD_HANDSHAKE(msg);
    }
    break;
    case POD_IMU1_STATUS:
    {
      handlePOD_IMU1_STATUS(msg);
    }
    break;
    case POD_IMU2_STATUS:
    {
      handlePOD_IMU2_STATUS(msg);
    }
    break;
    case POD_POWERTRAIN_STATUS:
    {
      handlePOD_POWERTRAIN_STATUS(msg);
    }
    break;
    case POD_REAR_ULTRASONIC_STATUS:
    {
      handlePOD_REAR_ULTRASONIC_STATUS(msg);
    }
    break;
    case POD_STATUS:
    {
      handlePOD_STATUS(msg);
    }
    break;
    case POD_VMS_SW_VER:
    {
      handlePOD_VMS_SW_VER(msg);
    }
    break;
    case POD_WDG_SW_VER:
    {
      handlePOD_WDG_SW_VER(msg);
    }
    break;
    case POD_WHL_CNT:
    {
      handlePOD_WHL_CNT(msg);
    }
    break;
    case POD_WHL_SW_VER:
    {
      handlePOD_WHL_SW_VER(msg);
    }
    break;
    default:
    {
      ROS_INFO("[%x] : [%d] : NOT DEFINED", msg.id, (int)sizeof(msg));
    }
    break;
  }
}

void PodRxNode::spin()
{
  ros::spin();
}

}
}
