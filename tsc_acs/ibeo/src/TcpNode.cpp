/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#include "ibeo/TcpNode.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include "BigEndian.h"
#include "../../version/version.h"

namespace tsc_acs {
namespace ibeo {

const unsigned char magicWord[4] = {0xAF, 0xFE, 0xC0, 0xC2};

TcpNode::TcpNode(const std::string & interface,
            const std::string & local_ip,
            const std::string & server_ip,
            int server_port, uint32_t maxDataSize,
            bool is_fusion)
                        : maxDataSize(maxDataSize)
{
	ReportVersion(node);

  fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd == -1)
  {
    ROS_ERROR("Failed to open UDP socket");
    throw "Failed to open UDP socket";
  }

  //---------------------------------
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
  sock.sin_port = htons(server_port);
  if (inet_aton(local_ip.c_str(), &sock.sin_addr) == 0)
  {
    sock.sin_addr.s_addr = INADDR_BROADCAST;
  }
  if (bind(fd, (sockaddr*)&sock, sizeof(sockaddr)) != 0)
  {
    ROS_ERROR("Failed to bind to socket");
    close(fd);
    throw "Failed to bind to socket";
  }
  //---------------------------------

  struct sockaddr_in sock_addr = {0};
  sock_addr.sin_family = AF_INET;
  sock_addr.sin_port = htons(server_port);
  struct hostent * pHost = gethostbyname(server_ip.c_str());
  memcpy(&sock_addr.sin_addr, pHost->h_addr, pHost->h_length);

  if (connect(fd, (struct sockaddr *)&sock_addr, sizeof(sock_addr)) < 0)
  {
    ROS_FATAL("Connection Failed");
    exit(-1);
  }


  data = new uint8_t[maxDataSize];
  if (data == NULL)
  {
    ROS_ERROR("Failed to allocate data memory");
    throw "Failed to allocate data memory";
  }

  // ---
  pub = node.advertise< ::ibeo::Message>("ibeo/message", 10);

  if (is_fusion)
  {
    // Request ALL data types
    setFilter(0x0000, 0xFFFF);
  }
}

TcpNode::~TcpNode()
{
  delete data;
  close(fd);
}
void TcpNode::spin()
{
  while(ros::ok())
  {
    findStartOfMessage();
    ::ibeo::Message msg;
    msg.header.stamp = ros::Time::now();
    receiveHeader(msg.ibeo_header);
    receiveData(msg.data, msg.ibeo_header.data_size);
    pub.publish(msg);
    ros::spinOnce();
  }
}

ssize_t TcpNode::receive(uint8_t * buf, size_t size)
{
  ssize_t bytes = recv(fd, buf, size, 0);
  return bytes;
}
template<typename T> T TcpNode::receiveBigEndian()
{
  uint8_t buf[sizeof(T)];
  receive(buf, sizeof(T));
  return fromBigEndian<T>(buf);
}
void TcpNode::findStartOfMessage()
{
  bool found = false;
  unsigned char buf[sizeof(magicWord)];
  int count = 0;
  do
  {
    receive(buf, sizeof(magicWord));
    count++;
  } while (memcmp(buf, magicWord, sizeof(magicWord)) != 0);

  if (count > 1)
  {
    ROS_INFO("Discarded %d reads looking for start of message.", count-1);
  }
}
void TcpNode::receiveHeader(::ibeo::IbeoHeader & header)
{
  header.size_prev = receiveBigEndian<uint32_t>();
  header.data_size = receiveBigEndian<uint32_t>();
  unsigned char reserved;
  receive(&reserved, 1);
  receive(&header.device_id, 1);
  header.data_type = receiveBigEndian<uint16_t>();

  uint8_t timestamp[8];
  receive(timestamp, 8);
  header.timestamp = timeFromBigEndianNTP64(timestamp);
}
void TcpNode::receiveData(::ibeo::Message::_data_type & msgData, uint32_t dataSize)
{
  uint32_t totalBytesRead = 0;

  while(totalBytesRead < dataSize)
  {
    uint32_t readSize = std::min(dataSize - totalBytesRead, maxDataSize);
    uint32_t bytesRead = receive(data, readSize);
    msgData.insert(msgData.end(), &data[0], &data[bytesRead]);
    totalBytesRead += bytesRead;
  }
}

template<int dataSize> void TcpNode::sendCommand(uint16_t cmd, const uint8_t *data)
{
  uint8_t buf[24+2+dataSize] ={0};
  memcpy(&buf[0], magicWord, sizeof(magicWord));
  toBigEndian<uint32_t>(2+dataSize, &buf[8]);
  toBigEndian<uint16_t>(0x2010, &buf[14]);
  toBigEndian<uint16_t>(cmd, &buf[24]);
  memcpy(&buf[26], data, dataSize);

  send(fd, buf, sizeof(buf), 0);
}

void TcpNode::setFilter(uint16_t start, uint16_t end)
{
  uint8_t data[6];
  toBigEndian<uint16_t>(2, &data[0]);
  toBigEndian<uint16_t>(0x0000, &data[2]);
  toBigEndian<uint16_t>(0xFFFF, &data[4]);
  sendCommand<6>(0x0005, data);

  ROS_INFO("Filter message sent to Fusion");
}

}
}
