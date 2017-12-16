/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
namespace tsc_acs {

class PointCloudFileNode
{
public:
  PointCloudFileNode()
  {
    sub = node.subscribe("ibeo/point_cloud", 10, &PointCloudFileNode::rxCallback, this);
    pub = node.advertise<sensor_msgs::PointCloud>("ibeo/pc", 10);
  }
  void spin()
  {
    ros::spin();
  }

private:
  void rxCallback(const ::sensor_msgs::PointCloud2 & msg)
  {
    float * pointData = (float*)(uint8_t *)&msg.data[0];
    int nSamples = msg.width;

    sensor_msgs::PointCloud pc;
    pc.header = msg.header;
    pc.points.resize(nSamples);

    for(int n=0; n<nSamples; n++)
    {
      pc.points[n].x = pointData[n*3];
      pc.points[n].y = pointData[n*3+1];
      pc.points[n].z = pointData[n*3+2];
    }

    pub.publish(pc);
  }

private:
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
};


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_file");

  tsc_acs::PointCloudFileNode node;
  node.spin();

  return 0;
}
