/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "acs/Goal.h"
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <acs/AutoDemandConfig.h>
#include "acs/Goal.h"

namespace tsc_acs {
namespace acs {

class AutoDemandNode
{
public:
  AutoDemandNode();

  void spin();

  void goalCallback(const ::acs::Goal & msg);
  void locationCallback(const ::acs::Pose2DStamped & msg);
  void configCallback(::acs::AutoDemandConfig & config, uint32_t level);

private:
  ros::NodeHandle node;
  ros::Subscriber subPath;
  ros::Subscriber subLocation;
  ros::Publisher pubDemand;
  ros::Publisher pubConfig;

  ::acs::Goal goal;

  double maxOffset2;    // m2
  double lateralGain;   // rad/m
  double headingGain;   // /m
  double speed;         // m/s
  double lowSpeed;      // m/s
  double fullSpeedCurvature;    // rad/m
  double lowSpeedCurvature;    // rad/m
  bool useCurvatureControl;

  dynamic_reconfigure::Server< ::acs::AutoDemandConfig> configServer;
};

}
}
