/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "udp/ClientNode.h"

//! udp_client ROS node entry point.

//! Creates and spins a ClientNode ROS node.
//! interface, ip_address, port, timeout and max_data_size are ROS parameters
//! that can be set using ROS; all are passed to the ClientNode
//! constructor.
int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_client");

    ros::NodeHandle nodeParam("~");

    std::string interface = "";
    nodeParam.getParam("interface", interface);

    std::string ip_addr = "";
    nodeParam.getParam("ip_address", ip_addr);

    int port = 3000;
    nodeParam.getParam("port", port);

    int timeout = 10;
    nodeParam.getParam("timeout", timeout);

    int maxDataSize = 16*1024;
    nodeParam.getParam("max_data_size", maxDataSize);

    tsc_acs::udp::ClientNode node(interface, ip_addr, port, timeout, maxDataSize);

    node.spin();

    return 0;
}
