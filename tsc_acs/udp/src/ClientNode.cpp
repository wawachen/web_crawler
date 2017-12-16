/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "udp/ClientNode.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include "udp/Packet.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace udp {

ClientNode::ClientNode(const std::string & interface,
                             const std::string & ip_addr,
                             int port, int timeout, int maxDataSize)
                : timeout(timeout), maxDataSize(maxDataSize)
{
	ReportVersion(node);
  // Open Socket
  fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd == -1)
  {
    ROS_ERROR("Failed to open UDP socket");
    throw "Failed to open UDP socket";
  }

  if (interface.length() > 0)
  {
    if (setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE, interface.c_str(), interface.length()) != 0)
    {
      ROS_ERROR("Failed to bind to interface: [%s]", interface.c_str());
      close(fd);
      throw "Failed to bind to interface";
    }
  }

  sockaddr_in sock;
  memset(&sock, 0, sizeof(sockaddr_in));
  sock.sin_family = AF_INET;
  sock.sin_port = htons(port);
  if (inet_aton(ip_addr.c_str(), &sock.sin_addr) == 0)
  {
    sock.sin_addr.s_addr = INADDR_BROADCAST;
  }
  if (bind(fd, (sockaddr*)&sock, sizeof(sockaddr)) != 0)
  {
    ROS_ERROR("Failed to bind to socket");
    close(fd);
    throw "Failed to bind to socket";
  }

  data = new uint8_t[maxDataSize];
  if (data == NULL)
  {
    ROS_ERROR("Failed to allocate data memory");
    throw "Failed to allocate data memory";
  }

  // ---
  pub = node.advertise< ::udp::Packet>("udp/rx", 1000);
}

ClientNode::~ClientNode()
{
  delete data;
  close(fd);
}

void ClientNode::spin()
{
  while(ros::ok())
  {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    struct timeval tv;
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;

    if (select(fd+1, &fds, NULL, NULL, &tv) > 0)
    {
      int bytesRead = recvfrom(fd, data, maxDataSize, 0, NULL, NULL);
      if (bytesRead > 0 && bytesRead <= maxDataSize)
      {
        ::udp::Packet msg;
        msg.header.stamp = ros::Time::now();
        msg.data.insert(msg.data.end(), &data[0], &data[bytesRead]);
        pub.publish(msg);
      }
    }
    ros::spinOnce();
  }
}


}
}
