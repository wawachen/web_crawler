/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "ros/ros.h"
#include "lutz/Version.h"
#include "lutz/PodCanId.h"

using namespace tsc_acs::lutz;

const int topicCount = 7;
const uint32_t versionId[] = {
    POD_EPB_SW_VER,
    POD_EPI_SW_VER,
    POD_EPO_SW_VER,
    POD_GWY_SW_VER,
    POD_VMS_SW_VER,
    POD_WDG_SW_VER,
    POD_WHL_SW_VER
};
const char * versionTopics[] = {
    "epb_ver",
    "epi_ver",
    "epo_ver",
    "gwy_ver",
    "vms_ver",
    "wdg_ver",
    "whl_ver"};

::lutz::Version versions[topicCount];

void rxCallback(const lutz::Version & msg)
{
  for(int i=0; i<topicCount; i++)
  {
    if(msg.pod_header.id == versionId[i])
    {
      versions[i] = msg;
      return;
    }
  }

  ROS_ERROR("Unexpected Version Id %X", msg.pod_header.id);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lutz_version");

    ros::NodeHandle node;

    ros::Subscriber sub[topicCount];

    for(int i=0; i<topicCount; i++)
      sub[i] = node.subscribe(versionTopics[i], 1, rxCallback);

    // All Versions should be latched, run for 1s
    ros::Rate rate(10); // 10Hz
    for(int i=0; i<10; i++)
    {
      ros::spinOnce();
      rate.sleep();
    }

    for(int i=0; i<topicCount; i++)
    {
      printf("%s %d.%d %c%c%c %d\n", versionTopics[i],
               versions[i].major_version,
               versions[i].minor_version,
               versions[i].id0,
               versions[i].id1,
               versions[i].id2,
               (int)versions[i].option);
    }

    return 0;
}
