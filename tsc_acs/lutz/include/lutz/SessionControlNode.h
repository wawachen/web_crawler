/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once
#include <lutz/PodStates.h>
#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include "lutz/PodHeader.h"
#include "pod/PodDemand.h"
#include "pod/AuxiliaryDemand.h"
#include "lutz/SessionControl.h"
#include "lutz/Status.h"
#include "lutz/Handshake.h"
#include "lutz/Battery.h"
#include "lutz/EPS.h"
#include "lutz/Steer.h"
#include "lutz/Powertrain.h"
#include "lutz/ParkBrake.h"
#include "lutz/ControlCommand1.h"
#include "lutz/UserInstructions.h"
#include "lutz/UICode.h"

namespace tsc_acs {
namespace lutz {

//! ROS Node for session control.

class SessionControlNode
{
public:
  //! Constructor
  SessionControlNode();

  //! Executes node activity
  void spin();

private:
  void rxCallback(const ::pod::PodDemand & msg);
  void handlePOD_STATUS(const ::lutz::Status & msg);
  void handlePOD_HANDSHAKE(const ::lutz::Handshake & msg);
  void handlePOD_BATTERY(const ::lutz::Battery & msg);
  void handlePOD_EPS(const ::lutz::EPS & msg);
  void handlePOD_EPS_INBOARD(const ::lutz::Steer & msg);
  void handlePOD_EPS_OUTBOARD(const ::lutz::Steer & msg);
  void handlePOD_POWERTRAIN(const ::lutz::Powertrain & msg);
  void handlePOD_PARK_BRAKE(const ::lutz::ParkBrake & msg);
  void handleACS_CTRL_CMD1(const ::lutz::ControlCommand1 & msg);
  void handleBumpStrips();
  void handlePublisher();

  bool MakeKeyManual();
  bool MakeKeyAuto();
  void MakeSession0();

  //session valid
  ReadyState GetSessionValidState();
  void MakeSessionValid(const ReadyState & reason);

  //primed valid
  ReadyState GetPrimedValidState();
  void MakePrimedValid(const ReadyState & reason);

  //autonomous valid
  ReadyState GetAutonomousValidState();
  void MakeAutonomousValid(const ReadyState & reason);

  //session cancelled
  ReadyState GetSessionCancelledState();
  void MakeSessionCancelled();


  ReadyState GetRevokeTrueState();
  void GetRevokeReason(const ReadyState & reason);
//  bool IsRevokeTrue();
  void GetRevokeReason2();

  //override
  ReadyState GetOverrideTrueState();
  void GetOverrideTrueReason(const ReadyState & reason);
  void MakeOverrideFalse(const ReadyState & reason);

  //error
  ReadyState GetErrorTrueState();
  void MakeErrorFalse(const ReadyState & reason);

  //estop
  ReadyState GetEstopState();
  void GetEstopReason(const ReadyState & reason);
  void MakeEstopFalse(const ReadyState & reason);

  //pod ready
  ReadyState GetPodReadyState();
  void MakePodReady(const ReadyState & reason);

  //stationary
  ReadyState GetStationaryState();
  void MakePodStationary(const ReadyState & reason);

  //handshake
  ReadyState GetHandshakeGoodState();
  void MakeHandshakeGood(const ReadyState & reason);

  //autonomy ready
  ReadyState GetAutonomyReadyState();
  void MakeAutonomyReady(const ReadyState & reason);

  void EnterAutonomous();
  void generateUserInstruction(const uint8_t uiTemp, const bool flag);

private:
  ros::NodeHandle node;
  ros::Subscriber subAutonomous_Request;
  ros::Subscriber subPOD_STATUS;
  ros::Subscriber subPOD_HANDSHAKE;
  ros::Subscriber subPOD_BATTERY;
  ros::Subscriber subPOD_EPS;
  ros::Subscriber subPOD_EPS_INBOARD;
  ros::Subscriber subPOD_EPS_OUTBOARD;
  ros::Subscriber subPOD_POWERTRAIN;
  ros::Subscriber subPOD_PARK_BRAKE;
  ros::Subscriber subACS_CTRL_CMD1;

  ros::Rate sendRate;

  ros::Publisher pubSession_Control;
  ros::Publisher pubUICode;

  uint8_t frameCount;
  uint8_t sessionId;
  uint8_t podStateRequest;
  uint8_t podState;
  bool autoRequest;
  bool laseDriveByWireRequest;
  bool frontBumper;
  bool rearBumper;

  ::lutz::Status statusMsg;
  ::lutz::Handshake handshakeMsg;
  ::lutz::Battery batteryMsg;
  ::lutz::EPS epsMsg;
  ::lutz::Steer epsInboardMsg;
  ::lutz::Steer epsOutboardMsg;
  ::lutz::Powertrain powertrainMsg;
  ::lutz::ParkBrake parkBrakeMsg;
  ::lutz::ControlCommand1 controlCommand1Msg;
  ReadyState tempReadyState;
  ReadyState tempReadyStateOld;

};

}
}
