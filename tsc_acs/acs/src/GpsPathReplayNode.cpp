/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "acs/Pose2to3.h"
#include "acs/GpsPathReplayNode.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <boost/assign/std/vector.hpp>
#include "acs/PathReplayCfg.h"
#include "../../version/version.h"

using namespace boost::assign;

namespace tsc_acs {
namespace acs {

GpsPathReplayNode::GpsPathReplayNode()
  : nextGoalIndex(0)
{
	ReportVersion(node);
	ros::NodeHandle nodeParam("~");

	std::string pathfile;
	nodeParam.getParam("pathfile", pathfile);

	std::string origin;
	nodeParam.getParam("origin", origin);


//	double lookAhead = lookAheadTime * ;
//	nodeParam.getParam("look_ahead", lookAhead);
//	lookAhead2 = lookAhead*lookAhead;

	if (origin.empty())
	{
		path.ReadFile(pathfile);
	}
	else
	{
		double indexComma = origin.find(",");
		if (indexComma == std::string::npos)
		{
			ROS_ERROR("Invalid origin parameter.");
		}
		else
		{
			double originLat = std::atof(origin.c_str());
			double originLng = std::atof(origin.substr(indexComma+1).c_str());
			path.ReadFile(pathfile, originLat, originLng);
		}
	}
  ROS_INFO("GpsPathReplay: Path %d", (int)path.size());

  pub = node.advertise< ::acs::Goal>("/acs/goal", 2, true);
  pub3 = node.advertise< geometry_msgs::PoseStamped>("/acs/goal3", 2, true);
  pubConfig = node.advertise< ::acs::PathReplayCfg>("/path_replay_node/config", 2, true);

	dynamic_reconfigure::Server< ::acs::PathReplayConfig>::CallbackType configCallbackFn =
		boost::bind(&GpsPathReplayNode::configCallback, this, _1, _2);
	configServer.setCallback(configCallbackFn);

  subReset = node.subscribe("/acs/path_reset", 3, &GpsPathReplayNode::resetCallback, this);

	std::vector<double> originParam;
	originParam += path.Origin().Latitude(), path.Origin().Longitude(), path.Origin().Heading();
	originParam += path.Origin().X(), path.Origin().Y(), path.Origin().Theta();
	node.setParam("/acs/origin", originParam);

	std::vector<double> startParam;
	startParam += path[0].Latitude(), path[0].Longitude(), path[0].Heading();
	startParam += path[0].X(), path[0].Y(), path[0].Theta();
	node.setParam("/acs/start", startParam);

  subLocation = node.subscribe("/acs/location", 3, &GpsPathReplayNode::locationCallback, this);

  pubPath = node.advertise< nav_msgs::Path>("allPath", 1, true);
  nav_msgs::Path pathMsg;
  pathMsg.header.frame_id = "map";
  pathMsg.header.stamp = ros::Time::now();
  for(GpsPath::const_iterator pnt=path.begin();
      pnt != path.end(); pnt++)
  {
    ::acs::Goal goal;
    pnt->Get(goal);
    pathMsg.poses.push_back(Pose2to3(goal.pose.pose, pathMsg.header.stamp));
  }
  pubPath.publish(pathMsg);
}

void GpsPathReplayNode::spin()
{
  ros::spin();
}

double GpsPathReplayNode::calculateLookAheadDistance(const ::acs::Goal & nextPoint)
{
  double lookAhead2 = nextPoint.max_speed * nextPoint.max_speed * loodAheadTime2;
  if (lookAhead2 >= maxLookAheadDistance2)
	  return maxLookAheadDistance2;
  else if (lookAhead2 <= minLookAheadDistance2)
	  return minLookAheadDistance2;
  else
	  return lookAhead2;
}

void GpsPathReplayNode::locationCallback(const ::acs::Pose2DStamped & msg)
{
  if (!path.empty())
  {
    if (reset)
    {
    	ROS_INFO("resetCalled");
      // Search for first point within look ahead range.
      int j = nextGoalIndex;
      bool found = false;

      while(!found && (j+1)%path.size() != nextGoalIndex)
      {
        ::acs::Goal nextPoint;
        path[j].Get(nextPoint);

        double lookAhead2 = maxLookAheadDistance2;
        double dr2 = (nextPoint.pose.pose.x - msg.pose.x)*(nextPoint.pose.pose.x - msg.pose.x) +
					(nextPoint.pose.pose.y - msg.pose.y)*(nextPoint.pose.pose.y - msg.pose.y);
        if (dr2 < lookAhead2)
        {
          found = true;
          nextGoalIndex = j;
        }

        j = (j+1) % path.size();

      }

      if (!found)
      {
        ROS_INFO("Failed to find point on path with range of point.");
      }

      reset = false;
    }

    // Publish next goal if in look ahead range
    bool sent;
    do
    {
      sent = false;
      ::acs::Goal nextPoint;
      nextPoint.pose.header.stamp = msg.header.stamp;
      path[nextGoalIndex].Get(nextPoint);

      double lookAhead2 = calculateLookAheadDistance(nextPoint);
      double dr2 = (nextPoint.pose.pose.x - msg.pose.x)*(nextPoint.pose.pose.x - msg.pose.x) +
          (nextPoint.pose.pose.y - msg.pose.y)*(nextPoint.pose.pose.y - msg.pose.y);
      if (dr2 < lookAhead2)
      {
        pub.publish(nextPoint);
        nextGoalIndex = (nextGoalIndex+1) % path.size();

        pub3.publish(Pose2to3(nextPoint.pose.pose, ros::Time::now()));
        sent = true;
      }
    } while (sent);
  }
}

void GpsPathReplayNode::resetCallback(const std_msgs::Header & msg)
{
  reset = true;
}

void GpsPathReplayNode::configCallback(::acs::PathReplayConfig & config, uint32_t level)
{
	double fullSpeed = config.speed;
	double lowSpeed = config.low_speed;
	double fullSpeedCurvature = config.full_speed_curvature;
	double lowSpeedCurvature = config.low_speed_curvature;
	double deceleration = config.deceleration;
	loodAheadTime2 = config.look_ahead_time * config.look_ahead_time;
	minLookAheadDistance2 = config.min_look_ahead_distance * config.min_look_ahead_distance;
	maxLookAheadDistance2 = config.max_look_ahead_distance * config.max_look_ahead_distance;

	path.CalculateMaxSpeed(fullSpeed, lowSpeed, fullSpeedCurvature, lowSpeedCurvature, deceleration);

	::acs::PathReplayCfg msgConfig;
	msgConfig.speed = config.speed;
	msgConfig.low_speed = config.low_speed;
	msgConfig.full_speed_curvature = config.full_speed_curvature;
	msgConfig.low_speed_curvature = config.low_speed_curvature;
	msgConfig.deceleration = config.deceleration;
	msgConfig.look_ahead_time = config.look_ahead_time;
	msgConfig.min_look_ahead_distance = config.min_look_ahead_distance;
	msgConfig.max_look_ahead_distance = config.max_look_ahead_distance;

	msgConfig.header.stamp = ::ros::Time::now();

	pubConfig.publish(msgConfig);

}

}
}
