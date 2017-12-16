/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "ibeo/Message.h"

namespace tsc_acs {
namespace ibeo {

//! Communication layer for Ibeo Fusion and Lux sensors

//! A ROS node that opens a TCP connection to an Ibeo device (sensor or ECU)
//! and listens for incoming messages.  The message header is read followed by
//! the payload, the size of which is defined in the header.  This is then
//1 published on a ROS topic - ibeo/message
class TcpNode
{
public:
  TcpNode(const std::string & interface,
          const std::string & local_ip,
          const std::string & server_ip,
          int server_port, uint32_t maxDataSize,
          bool is_fusion);
  ~TcpNode();
  void spin();

private:
  ssize_t receive(uint8_t * buf, size_t size);
  template<typename T> T receiveBigEndian();
  void findStartOfMessage();
  void receiveHeader(::ibeo::IbeoHeader & header);
  void receiveData(::ibeo::Message::_data_type & msgData, uint32_t dataSize);
  template<int dataSize> void sendCommand(uint16_t cmd, const uint8_t *data);
  void setFilter(uint16_t start, uint16_t end);

private:
  ros::NodeHandle node;
  ros::Publisher pub;

  int fd;
  uint8_t * data;
  uint32_t maxDataSize;
};

}
}
