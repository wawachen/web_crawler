/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"

geometry_msgs::PoseStamped Pose2to3(const geometry_msgs::Pose2D & pose, const ros::Time & stamp)
{
  geometry_msgs::PoseStamped pose3;
  pose3.pose.position.x = pose.x;
  pose3.pose.position.y = pose.y;
  pose3.pose.position.z = 0;
  pose3.pose.orientation.x = 0;
  pose3.pose.orientation.y = 0;
  pose3.pose.orientation.z = std::sin(pose.theta/2);
  pose3.pose.orientation.w = std::cos(pose.theta/2);
  pose3.header.stamp = stamp;
  pose3.header.frame_id = "map";
  return pose3;
}
