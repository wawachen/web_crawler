/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "ibeo/TcpNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ibeoMessage");

    ros::NodeHandle nodeParam("~");

    std::string interface = "";
    nodeParam.getParam("interface", interface);

    std::string local_ip = "";
    nodeParam.getParam("local_ip", local_ip);

    std::string server_ip = "192.168.102.100";
    nodeParam.getParam("server_ip", server_ip);

    int server_port = 12002;
    nodeParam.getParam("server_port", server_port);

    int maxDataSize = 16*1024;
    nodeParam.getParam("max_data_size", maxDataSize);

    bool is_fusion = true;
    nodeParam.getParam("is_fusion", is_fusion);

    tsc_acs::ibeo::TcpNode node(interface, local_ip, server_ip, server_port,
                                (uint32_t)maxDataSize, is_fusion);

    node.spin();

    return 0;
}
