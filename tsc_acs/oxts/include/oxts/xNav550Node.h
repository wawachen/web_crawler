/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"
#include "oxts/oxts_dispatch.h"
#include "udp/Packet.h"

namespace tsc_acs {
namespace oxts {

//! ROS Node for listening to incoming data from an OXTS xNav550 GPS/INS

//! This node listens for UDP packets from the device published
//! on the udp/rx topic.  Data from the packets are published on the standard
//! gps/fix, gps/vel, imu/data and gps/odom topics.
class xNav550Node
{
public:
  //! Constructor

  //! Subscribes to the udp/rx topic and advertises on the gps/fix, gps/vel,
  //! imu/data and gps/odom topics.
  //! \param frame_id ROS frame identifier for the gps/fix and imu/data
  //!   messages.
  //! \param frame_id_vel ROS frame identifier for the gps/vel and gps/odom
  //!   messages.
  xNav550Node(const std::string & frame_id, const std::string & frame_id_vel);

  //! Executes node activity
  void spin();

private:
  void rxCallback(const udp::Packet & msg);
  static inline double SQUARE(double x);
  inline bool validatePacket(const Packet & packet);
  inline void handlePacket(const Packet & packet, const ros::Time & stamp);

  void handleGnssInfo(const GnssInfo & gnssInfo, const ros::Time & stamp);
  void handleKalmanInnovation1(const KalmanInnovation1 & innov, const ros::Time & stamp);
  void handlePrimaryReceiverInfo(const ReceiverInfo & info, const ros::Time & stamp);
  void handleSecondaryReceiverInfo(const ReceiverInfo & info, const ros::Time & stamp);
  void handleReceiverInfo(ros::Publisher & pub, const ReceiverInfo & info, const ros::Time & stamp);
  void handlePositionAccuracy(const Position & positionAccuracy);
  void handleVelocityAccuracy(const Position & velocityAccuracy);
  void handleOrientationAccuracy(const Orientation & orientationAccuracy);
  void handlePrimaryAntennaPosition(const XYZ & position, const ros::Time & stamp);
  void handleOrientationDual(const Polar & orientation, const ros::Time & stamp);
  void handlePrimaryAntennaPositionAccuracy(const XYZ & positionAccuracy, const ros::Time & stamp);
  void handleOrientationDualAccuracy(const Polar & orientationAccuracy, const ros::Time & stamp);

private:
  ros::NodeHandle node;

  ros::Subscriber sub;

  ros::Publisher pub_fix;
  ros::Publisher pub_vel;
  ros::Publisher pub_imu;
  ros::Publisher pub_gnssInfo;
  ros::Publisher pub_kalmanInnovation;
  ros::Publisher pub_primaryReceiverInfo;
  ros::Publisher pub_secondaryReceiverInfo;
  ros::Publisher pub_antenna_position;
  ros::Publisher pub_batch_b;

  bool first;

  std::string frame_id;
  std::string frame_id_vel;

  uint8_t fix_status;

  uint8_t position_covariance_type;
  double position_covariance[3];
  uint8_t velocity_covariance_type;
  double velocity_covariance[3];
  uint8_t orientation_covariance_type;
  double orientation_covariance[3];

  XYZ primaryAntennaPosition;
  XYZ primaryAntennaPositionAccuracy;
  Polar orientationDual;
  Polar orientationDualAccuracy;
};

}
}
