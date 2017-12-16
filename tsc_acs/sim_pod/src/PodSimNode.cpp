/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "sim_pod/PodSimNode.h"
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "../../acs/include/acs/Pose2to3.h"
#include "oxts/BatchB.h"
#include <unistd.h>
#include <vector>
#include "../../version/version.h"

namespace tsc_acs {
namespace sim_pod {

PodSimNode::PodSimNode(double steerAccel, double speedAccel)
  :steerAccel(steerAccel), speedAccel(speedAccel),
   steer(0), speed(0)
{
  pose.x = 0;
  pose.y = 0;
  pose.theta = -0.95;

  previous.speed = -1000000;
  previous.steer = 0;

  subDemand = node.subscribe("pod_demand", 3, &PodSimNode::demandCallback, this);

  pubLocation = node.advertise< ::geometry_msgs::Pose2D>("sim/location", 2, true);
  pubLocation3 = node.advertise< ::geometry_msgs::PoseStamped>("sim/location3", 2);
  pubGps = node.advertise< ::oxts::BatchB>("sim/batchB",2);

	ReportVersion(node);

	while (!node.hasParam("/acs/origin"))
	{
		ROS_INFO("Waiting for /acs/origin");
		usleep(1000000);    // Sleep 1s
	}

	while (!node.hasParam("/acs/start"))
	{
		ROS_INFO("Waiting for /acs/start");
		usleep(1000000);    // Sleep 1s
	}

	std::vector<double> origin_in;
	node.getParam("/acs/origin", origin_in);
	ROS_INFO("Origin %2.4f, %2.4f, %2.4f : %2.4f, %2.4f, %2.4f",
		origin_in[0], origin_in[1], origin_in[2],
		origin_in[3], origin_in[4], origin_in[5]);

  origin.Set(origin_in[0], origin_in[1], origin_in[2],
             origin_in[3], origin_in[4], origin_in[5]);

	std::vector<double> start_in;
	node.getParam("/acs/start", start_in);
	ROS_INFO("Start %2.4f, %2.4f, %2.4f : %2.4f, %2.4f, %2.4f",
		start_in[0], start_in[1], start_in[2],
		start_in[3], start_in[4], start_in[5]);

	pose.x = start_in[3];
	pose.y = start_in[4];
	pose.theta = start_in[5];
}

void PodSimNode::spin()
{
  ros::Rate rate(50);

  while(ros::ok())
  {
    ros::Time now = ros::Time::now();

    pubLocation.publish(pose);
    pubLocation3.publish(Pose2to3(pose, now));

    ::oxts::BatchB batchB;
    batchB.header.stamp = now;
    origin.ToLatLng(pose.x, pose.y, pose.theta, batchB.latitude, batchB.longitude, batchB.heading);
    batchB.altitude = 0;
    batchB.vel_north = 0;
    batchB.vel_east = 0;
    batchB.vel_down = 0;
    batchB.pitch = 0;
    batchB.roll = 0;
    pubGps.publish(batchB);

    ros::spinOnce();
    rate.sleep();
  }
}

void PodSimNode::demandCallback(const ::pod::PodDemand & msg)
{
  if (previous.speed != -1000000)
  {
    double dT = (msg.header.stamp - previous.header.stamp).toSec();
    if (dT > 0)
    {
      if (dT > 0.100)
      {
        dT = 0.01; // Allow for processor being busy - slow down simulation if updates < 10Hz
      }

      double dSpeed = previous.speed - speed;
      double dSteer = previous.steer - steer;

      if (dSpeed > speedAccel * dT)
        dSpeed = speedAccel * dT;
      else if (dSpeed < -speedAccel * dT)
        dSpeed = -speedAccel * dT;
      speed += dSpeed;

      if (dSteer > steerAccel * dT)
        dSteer = steerAccel * dT;
      else if (dSteer < -steerAccel * dT)
        dSteer = -steerAccel * dT;
      steer += dSteer;

      double dS = speed * dT;      // Distance travelled.
      double dTheta = steer * dS;

      double dF, dL;
      if (previous.steer > 1e-3)
      {
        double R = 1.0 / steer;   // Radius of curvature.
        double r = 2 * R * std::sin(dTheta/2);    // Displacement

        dF = r * std::cos(dTheta/2);   // Forward movement from previous
        dL = r * std::sin(dTheta/2);   // Lateral movement from previous
      }
      else
      {
        dF = dS;
        dL = 0;
      }

      double dX = dF * std::cos(pose.theta) - dL * std::sin(pose.theta);
      double dY = dF * std::sin(pose.theta) + dL * std::cos(pose.theta);

      pose.x += dX;
      pose.y += dY;
      pose.theta = fmod(pose.theta + dTheta + 3 * M_PI, 2 * M_PI) - M_PI;
    }
    else
    {
        ROS_ERROR("Negative delta time! %2.3f", dT);
        ROS_ERROR("msg.header.stamp %2.3f", msg.header.stamp.toSec());
        ROS_ERROR("previous.header.stamp %2.3f", previous.header.stamp.toSec());
    }
  }

  previous = msg;

  ROS_INFO("Demand %2.3f\t%2.3f\tPose %2.3f, %2.3f : %2.3f",
           msg.speed, msg.steer,
           pose.x, pose.y, pose.theta);
}

}
}
