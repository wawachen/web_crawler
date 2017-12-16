/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "acs/AutoDemandNode.h"
#include "pod/PodDemandSource.h"
#include "acs/AutoDemandCfg.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace acs {

AutoDemandNode::AutoDemandNode()
  : maxOffset2(0), lateralGain(0), headingGain(0), speed(0),
    lowSpeed(0), fullSpeedCurvature(0), lowSpeedCurvature(1000), useCurvatureControl(false)
{
	ReportVersion(node);
  pubDemand = node.advertise< ::pod::PodDemandSource>("/acs/demand", 2);
  pubConfig = node.advertise< ::acs::AutoDemandCfg>("/acs/config", 2, true);

  goal.pose.pose.x = -1000000;
  dynamic_reconfigure::Server< ::acs::AutoDemandConfig>::CallbackType configCallbackFn =
      boost::bind(&AutoDemandNode::configCallback, this, _1, _2);
  configServer.setCallback(configCallbackFn);

  subPath = node.subscribe("/acs/goal", 3, &AutoDemandNode::goalCallback, this);
  subLocation = node.subscribe("/acs/location", 3, &AutoDemandNode::locationCallback, this);
}

void AutoDemandNode::spin()
{
  ros::spin();
}

void AutoDemandNode::configCallback(::acs::AutoDemandConfig & config, uint32_t level)
{
  maxOffset2 = config.max_offset*config.max_offset;
  lateralGain = config.lateral_gain;
  headingGain = config.heading_gain;
  speed = config.speed;
  lowSpeed = config.low_speed;
  fullSpeedCurvature = config.full_speed_curvature;
  lowSpeedCurvature = config.low_speed_curvature;
  useCurvatureControl = config.use_curvature_control;

  ::acs::AutoDemandCfg msgConfig;
  msgConfig.max_offset = config.max_offset;
  msgConfig.lateral_gain = config.lateral_gain;
  msgConfig.heading_gain = config.heading_gain;
  msgConfig.speed = config.speed;
  msgConfig.low_speed = config.low_speed;
  msgConfig.full_speed_curvature = config.full_speed_curvature;
  msgConfig.low_speed_curvature = config.low_speed_curvature;
  msgConfig.use_curvature_control = config.use_curvature_control;
  msgConfig.header.stamp = ::ros::Time::now();

  pubConfig.publish(msgConfig);
}

void AutoDemandNode::goalCallback(const ::acs::Goal & msg)
{
  goal = msg;
}

void AutoDemandNode::locationCallback(const ::acs::Pose2DStamped & msg)
{
  ::pod::PodDemandSource demand;
  double dx = goal.pose.pose.x - msg.pose.x;
  double dy = goal.pose.pose.y - msg.pose.y;
  double dr2 = dx * dx + dy * dy;
  if (dr2 < maxOffset2)
  {
    double eLong = dx * cos(msg.pose.theta) + dy * sin(msg.pose.theta);
    double eLat = -dx * sin(msg.pose.theta) + dy * cos(msg.pose.theta);
    double eHead = fmod(goal.pose.pose.theta - msg.pose.theta + M_PI, 2 * M_PI) - M_PI;

    double dr = sqrt(dr2);
    demand.podDemand.steer = eLat * lateralGain / dr + eHead * headingGain + goal.curvature*(useCurvatureControl?1:0);
    double magSteer = fabs(demand.podDemand.steer);
    if (magSteer < fullSpeedCurvature)
    {
      demand.podDemand.speed = speed;
    }
    else if (magSteer > lowSpeedCurvature)
    {
      demand.podDemand.speed = lowSpeed;
    }
    else
    {
      demand.podDemand.speed = lowSpeed +
          (magSteer - lowSpeedCurvature) * (speed - lowSpeed) /
            (fullSpeedCurvature - lowSpeedCurvature);
    }

		if (demand.podDemand.speed > goal.max_speed)
		{
			demand.podDemand.speed = goal.max_speed;
		}
  }
  else
  {
    demand.podDemand.steer = 0;
    demand.podDemand.speed = 0;
  }

  demand.podDemand.header.stamp = ros::Time::now();
  demand.podDemand.source = "auto_demand";
  demand.podDemand.driveByWireRequest = true;
  demand.abdicate = false;
  pubDemand.publish(demand);
}

}
}
