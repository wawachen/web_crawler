/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"

namespace tsc_acs {
namespace udp {

//! ROS Node for listening to incoming UDP packets.

//! This node opens a UDP port and publishes a message on the udp/rx ROS
//! topic.
class ClientNode
{
public:
  //! Constructor

  //! Opens the specified UDP port and advertises the udp/rx ROS topic.
  //! \param interface   Ethernet interface on which to open the port.  All
  //!    interfaces if empty string.
  //! \param ip_addr     IP address on which to open the port.  All IP
  //!    addresses if empty string.
  //! \param port        Port to open.
  //! \param timeout     Maximum time to wait (in ms) for each read.
  //! \param maxDataSize Maximum size (in bytes) of a data packet.
  //!   Beyond this size packets are not published.
  ClientNode(const std::string & interface, const std::string & ip_addr,
                int port, int timeout, int maxDataSize);

  //! Destructor
  ~ClientNode();

  //! Executes node activity

  //! Continually monitors UDP port for incoming packets.  Publishes all
  //! packets with a size more than zero and up to maxDataSize on the udp/rx
  //! topic.
  void spin();

private:
  ros::NodeHandle node;
  ros::Publisher pub;

  int fd;
  uint8_t * data;
  int timeout;
  int maxDataSize;
};

}
}
