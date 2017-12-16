/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <fstream>

namespace tsc_acs {

class PointCloudFileNode
{
public:
  PointCloudFileNode()
  {
    sub = node.subscribe("ibeo/point_cloud", 10, &PointCloudFileNode::rxCallback, this);
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

    char filename[1024];
    sprintf(filename, "cloud.%d.%03d.csv", msg.header.stamp.sec, msg.header.stamp.nsec/1000000);

    ROS_INFO("%s : %d", filename, nSamples);

    std::ofstream file;
    file.open (filename);

    for (int n=0; n<nSamples; n++)
    {
      file << pointData[n*3] << ','
          << pointData[n*3+1] << ','
          << pointData[n*3+2] << std::endl;
    }

    file.close();
  }

private:
  ros::NodeHandle node;
  ros::Subscriber sub;
};


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_file");

  tsc_acs::PointCloudFileNode node;
  node.spin();

  return 0;
}
