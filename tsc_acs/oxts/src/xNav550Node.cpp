/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

// ========================================================
// fragments of this code are taken from
// https://bitbucket.org/DataspeedInc/oxford_gps_eth
// and so are copyright Dataspeed Inc. under BSD License.
//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2015-2016, Dataspeed Inc.
//  All rights reserved.
// ========================================================

#include "oxts/xNav550Node.h"

// ROS messages
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include "oxts/GnssInfo.h"
#include "oxts/KalmanInnovation.h"
#include "oxts/ReceiverInfo.h"
#include "oxts/AntennaPosition.h"
#include "oxts/BatchB.h"
#include "../../version/version.h"

// Tf Quaternions
#include <tf/LinearMath/Quaternion.h>

namespace tsc_acs {
namespace oxts {

xNav550Node::xNav550Node(const std::string & frame_id, const std::string & frame_id_vel)
: first(true), frame_id(frame_id), frame_id_vel(frame_id_vel),
  fix_status(sensor_msgs::NavSatStatus::STATUS_FIX),
  position_covariance_type(sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN),
  velocity_covariance_type(sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN),
  orientation_covariance_type(sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
{
	ReportVersion(node);
  sub = node.subscribe("udp/rx", 10, &xNav550Node::rxCallback, this);

  pub_fix = node.advertise<sensor_msgs::NavSatFix>("gps/fix", 2);
  pub_vel = node.advertise<geometry_msgs::TwistWithCovarianceStamped>("gps/vel", 2);
  pub_imu = node.advertise<sensor_msgs::Imu>("imu/data", 2);
//  pub_odom = node.advertise<nav_msgs::Odometry>("gps/odom", 2);
  pub_gnssInfo = node.advertise< ::oxts::GnssInfo>("oxts/gnss_info", 2);
  pub_kalmanInnovation = node.advertise< ::oxts::KalmanInnovation>("oxts/kalman_innov", 2);
  pub_primaryReceiverInfo = node.advertise< ::oxts::ReceiverInfo>("oxts/primary_rx_info", 2);
  pub_secondaryReceiverInfo = node.advertise< ::oxts::ReceiverInfo>("oxts/secondary_rx_info", 2);
  pub_antenna_position = node.advertise< ::oxts::AntennaPosition>("oxts/antenna_position", 2);
  pub_batch_b = node.advertise< ::oxts::BatchB>("oxts/batchB", 2);
}

void xNav550Node::spin()
{
  ros::spin();
}

void xNav550Node::rxCallback(const ::udp::Packet & msg)
{
  const Packet * packet = (const Packet *)msg.data.data();
  if (validatePacket(*packet))
  {
    if (first)
    {
      first = false;
      ROS_INFO("Connected to Oxford GPS");// at %s:%u", inet_ntoa(((sockaddr_in*)&source)->sin_addr), htons(((sockaddr_in*)&source)->sin_port));
    }
    handlePacket(*packet, msg.header.stamp);
  }
}

double xNav550Node::SQUARE(double x)
{
  return x * x;
}

bool xNav550Node::validatePacket(const Packet & packet)
{
  if (packet.sync == 0xE7) {
    const uint8_t *ptr = (uint8_t*)&packet;
    uint8_t chksum = 0;
    for (unsigned int i = 1; i < sizeof(Packet) - 1; i++) {
      chksum += ptr[i];
    }
    return chksum == packet.chksum3;
  }
  return false;
}

void xNav550Node::handleGnssInfo(const GnssInfo & gnssInfo, const ros::Time & stamp)
{
  switch (gnssInfo.position_mode) {
    case MODE_DIFFERENTIAL:
    case MODE_DIFFERENTIAL_PP:
    case MODE_RTK_FLOAT:
    case MODE_RTK_INTEGER:
    case MODE_RTK_FLOAT_PP:
    case MODE_RTK_INTEGER_PP:
    case MODE_DOPLER_PP:
    case MODE_SPS_PP:
      fix_status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
      break;
    case MODE_OMNISTAR_VBS:
    case MODE_OMNISTAR_HP:
    case MODE_OMNISTAR_XP:
    case MODE_WAAS:
    case MODE_CDGPS:
      fix_status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
      break;
    case MODE_SPS:
      fix_status = sensor_msgs::NavSatStatus::STATUS_FIX;
      break;
    case MODE_NONE:
    case MODE_SEARCH:
    case MODE_DOPLER:
    case MODE_NO_DATA:
    case MODE_BLANKED:
    case MODE_NOT_RECOGNISED:
    case MODE_UNKNOWN:
    default:
      fix_status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
      break;
  }

  ::oxts::GnssInfo msg_gnssInfo;
  msg_gnssInfo.header.stamp = stamp;
  msg_gnssInfo.header.frame_id = frame_id;
  msg_gnssInfo.gps_minutes = gnssInfo.gps_minutes;
  msg_gnssInfo.satellites_tracks = gnssInfo.num_sats;
  msg_gnssInfo.position_mode = gnssInfo.position_mode;
  msg_gnssInfo.velocity_mode = gnssInfo.velocity_mode;
  msg_gnssInfo.orientation_mode = gnssInfo.orientation_mode;
  pub_gnssInfo.publish(msg_gnssInfo);
}

static inline float innovationFromByte(uint8_t value)
{
  return (value & 0x80 == 0) ? NAN : ((value & 0x7f) * 0.1f);
}

void xNav550Node::handleKalmanInnovation1(const KalmanInnovation1 & innov, const ros::Time & stamp)
{
  ::oxts::KalmanInnovation msg_innov;
  msg_innov.header.stamp = stamp;
  msg_innov.header.frame_id = frame_id;
  msg_innov.x = innovationFromByte(innov.x);
  msg_innov.y = innovationFromByte(innov.y);
  msg_innov.z = innovationFromByte(innov.z);
  msg_innov.vel_x = innovationFromByte(innov.vel_x);
  msg_innov.vel_y = innovationFromByte(innov.vel_y);
  msg_innov.vel_z = innovationFromByte(innov.vel_z);
  msg_innov.pitch = innovationFromByte(innov.pitch);
  msg_innov.heading = innovationFromByte(innov.heading);
  pub_kalmanInnovation.publish(msg_innov);
}

void xNav550Node::handlePrimaryReceiverInfo(const ReceiverInfo & info, const ros::Time & stamp)
{
  handleReceiverInfo(pub_primaryReceiverInfo, info, stamp);
}
void xNav550Node::handleSecondaryReceiverInfo(const ReceiverInfo & info, const ros::Time & stamp)
{
  handleReceiverInfo(pub_secondaryReceiverInfo, info, stamp);
}
void xNav550Node::handleReceiverInfo(ros::Publisher & pub, const ReceiverInfo & info, const ros::Time & stamp)
{
  ::oxts::ReceiverInfo msg_info;
  msg_info.header.stamp = stamp;
  msg_info.header.frame_id = frame_id;
  msg_info.byte_count = info.byte_count;
  msg_info.packet_count = info.packet_count;
  msg_info.bad_bytes = info.bad_bytes;
  msg_info.old_packets = info.old_packets;
  pub.publish(msg_info);
}

void xNav550Node::handlePositionAccuracy(const Position & positionAccuracy)
{
  if (positionAccuracy.age < 150) {
    position_covariance[0] = SQUARE((double)positionAccuracy.east * 1e-3);
    position_covariance[1] = SQUARE((double)positionAccuracy.north * 1e-3);
    position_covariance[2] = SQUARE((double)positionAccuracy.down * 1e-3);
    position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  } else {
    position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }
}

void xNav550Node::handleVelocityAccuracy(const Position & velocityAccuracy)
{
  if (velocityAccuracy.age < 150) {
    velocity_covariance[0] = SQUARE((double)velocityAccuracy.east * 1e-3);
    velocity_covariance[1] = SQUARE((double)velocityAccuracy.north * 1e-3);
    velocity_covariance[2] = SQUARE((double)velocityAccuracy.down * 1e-3);
    velocity_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  } else {
    velocity_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }
}

void xNav550Node::handleOrientationAccuracy(const Orientation & orientationAccuracy)
{
  if (orientationAccuracy.age < 150) {
    orientation_covariance[0] = SQUARE((double)orientationAccuracy.roll * 1e-5);
    orientation_covariance[1] = SQUARE((double)orientationAccuracy.pitch * 1e-5);
    orientation_covariance[2] = SQUARE((double)orientationAccuracy.heading * 1e-5);
    orientation_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  } else {
    orientation_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }
}

void xNav550Node::handlePrimaryAntennaPosition(const XYZ & position, const ros::Time & stamp)
{
  primaryAntennaPosition = position;
}
void xNav550Node::handleOrientationDual(const Polar & orientation, const ros::Time & stamp)
{
  orientationDual = orientation;
}
void xNav550Node::handlePrimaryAntennaPositionAccuracy(const XYZ & positionAccuracy, const ros::Time & stamp)
{
  primaryAntennaPositionAccuracy = positionAccuracy;
}
void xNav550Node::handleOrientationDualAccuracy(const Polar & orientationAccuracy, const ros::Time & stamp)
{
  orientationDualAccuracy = orientationAccuracy;

  if (orientationDualAccuracy.age < 150 &&
      primaryAntennaPosition.age < 150 &&
      orientationDual.age < 150 &&
      primaryAntennaPositionAccuracy.age < 150)
  {
    ::oxts::AntennaPosition msg_pos;
    msg_pos.header.stamp = stamp;
    msg_pos.header.frame_id = frame_id;
    msg_pos.x_primary = primaryAntennaPosition.x * 1e-3f;
    msg_pos.y_primary = primaryAntennaPosition.y * 1e-3f;
    msg_pos.z_primary = primaryAntennaPosition.z * 1e-3f;
    msg_pos.heading = orientationDual.heading * 1e-4f;
    msg_pos.pitch = orientationDual.pitch * 1e-4f;
    msg_pos.distance = orientationDual.distance * 1e-3f;
    msg_pos.x_primary_accuracy = primaryAntennaPositionAccuracy.x * 1e-3f;
    msg_pos.y_primary_accuracy = primaryAntennaPositionAccuracy.y * 1e-3f;
    msg_pos.z_primary_accuracy = primaryAntennaPositionAccuracy.z * 1e-3f;
    msg_pos.heading_accuracy = orientationDualAccuracy.heading * 1e-4f;
    msg_pos.pitch_accuracy = orientationDualAccuracy.pitch * 1e-4f;
    msg_pos.distance_accuracy = orientationDualAccuracy.distance * 1e-3f;

    pub_antenna_position.publish(msg_pos);
  }
}

void xNav550Node::handlePacket(const Packet & packet, const ros::Time & stamp)
{
  if (true) {   //(packet.nav_status == 4) {
    switch (packet.channel) {
      case CH_GNSS_INFO:            // 0
        handleGnssInfo(packet.chan.gnssInfo, stamp);
        break;
      case CH_KALMAN_INNOVATION_1:  // 1
        handleKalmanInnovation1(packet.chan.kalmanInnovation1, stamp);
        break;
      case CH_PRIMARY_RECEIVER_INFO:    // 2
        handlePrimaryReceiverInfo(packet.chan.receiverInfo, stamp);
        break;
      case CH_POSITION_ACCURACY:    // 3
        handlePositionAccuracy(packet.chan.position);
        break;
      case CH_VELOCITY_ACCURACY:    // 4
        handleVelocityAccuracy(packet.chan.position);
        break;
      case CH_ORIENTATION_ACCURACY: // 5
        handleOrientationAccuracy(packet.chan.orientation);
        break;
      case CH_PRIMARY_ANTENNA_POSTION:              // 12
        handlePrimaryAntennaPosition(packet.chan.xyz, stamp);
        break;
      case CH_ORIENTATION_DUAL:                     // 13
        handleOrientationDual(packet.chan.polar, stamp);
        break;
      case CH_PRIMARY_ANTENNA_POSITION_ACCURACY:    // 14
        handlePrimaryAntennaPositionAccuracy(packet.chan.xyz, stamp);
        break;
      case CH_ORIENTATION_DUAL_ACCURACY:            // 15
        handleOrientationDualAccuracy(packet.chan.polar, stamp);
        break;
      case CH_SECONDARY_RECEIVER_INFO:  // 17
        handleSecondaryReceiverInfo(packet.chan.receiverInfo, stamp);
        break;
    }

    sensor_msgs::NavSatFix msg_fix;
    msg_fix.header.stamp = stamp;
    msg_fix.header.frame_id = frame_id;
    msg_fix.latitude = packet.latitude * (180 / M_PI);
    msg_fix.longitude = packet.longitude * (180 / M_PI);
    msg_fix.altitude = packet.altitude;
    msg_fix.status.status = fix_status;
    msg_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    msg_fix.position_covariance_type = position_covariance_type;
    if (position_covariance_type > sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      msg_fix.position_covariance[0] = position_covariance[0]; // x
      msg_fix.position_covariance[4] = position_covariance[1]; // y
      msg_fix.position_covariance[8] = position_covariance[2]; // z
    }
    pub_fix.publish(msg_fix);

    geometry_msgs::TwistWithCovarianceStamped msg_vel;
    msg_vel.header.stamp = stamp;
    msg_vel.header.frame_id = frame_id_vel;
    msg_vel.twist.twist.linear.x = (double)packet.vel_north * 1e-4;
    msg_vel.twist.twist.linear.y = (double)packet.vel_east * -1e-4;
    msg_vel.twist.twist.linear.z = (double)packet.vel_down * -1e-4;
    if (velocity_covariance_type > sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      msg_vel.twist.covariance[0] = velocity_covariance[0]; // x
      msg_vel.twist.covariance[7] = velocity_covariance[1]; // y
      msg_vel.twist.covariance[14] = velocity_covariance[2]; // z
    }
    pub_vel.publish(msg_vel);

    tf::Quaternion q;
    q.setRPY((double)packet.roll * 1e-6, (double)packet.pitch * 1e-6, (double)packet.heading * -1e-6);
    sensor_msgs::Imu msg_imu;
    msg_imu.header.stamp = stamp;
    msg_imu.header.frame_id = frame_id;
    msg_imu.linear_acceleration.x = (double)packet.accel_x * 1e-4;
    msg_imu.linear_acceleration.y = (double)packet.accel_y * 1e-4;
    msg_imu.linear_acceleration.z = (double)packet.accel_z * -1e-4;
    msg_imu.linear_acceleration_covariance[0] = -1;
    msg_imu.angular_velocity.x = (double)packet.gyro_x * 1e-5;
    msg_imu.angular_velocity.y = (double)packet.gyro_y * 1e-5;
    msg_imu.angular_velocity.z = (double)packet.gyro_z * -1e-5;
    msg_imu.orientation.w = q.w();
    msg_imu.orientation.x = q.x();
    msg_imu.orientation.y = q.y();
    msg_imu.orientation.z = q.z();
    if (orientation_covariance_type > sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      msg_imu.orientation_covariance[0] = orientation_covariance[0]; // x
      msg_imu.orientation_covariance[4] = orientation_covariance[1]; // y
      msg_imu.orientation_covariance[8] = orientation_covariance[2]; // z
    } else {
      msg_imu.orientation_covariance[0] = 0.0174532925; // x
      msg_imu.orientation_covariance[4] = 0.0174532925; // y
      msg_imu.orientation_covariance[8] = 0.0174532925; // z
    }
    msg_imu.angular_velocity_covariance[0] = 0.000436332313; // x
    msg_imu.angular_velocity_covariance[4] = 0.000436332313; // y
    msg_imu.angular_velocity_covariance[8] = 0.000436332313; // x
    msg_imu.linear_acceleration_covariance[0] = 0.0004; // x
    msg_imu.linear_acceleration_covariance[4] = 0.0004; // y
    msg_imu.linear_acceleration_covariance[8] = 0.0004; // z
    pub_imu.publish(msg_imu);

    ::oxts::BatchB msg_batch_b;
    msg_batch_b.header.stamp = stamp;
    msg_batch_b.header.frame_id = frame_id;
    msg_batch_b.latitude = packet.latitude * (180 / M_PI);
    msg_batch_b.longitude = packet.longitude * (180 / M_PI);
    msg_batch_b.altitude = packet.altitude;
    msg_batch_b.vel_north = packet.vel_north * 1e-4;
    msg_batch_b.vel_east = packet.vel_east * 1e-4;
    msg_batch_b.vel_down = packet.vel_down * 1e-4;
    msg_batch_b.heading = packet.heading * 1e-6;
    msg_batch_b.pitch = packet.pitch * 1e-6;
    msg_batch_b.roll = packet.roll * 1e-6;
    pub_batch_b.publish(msg_batch_b);
  }
}

}
}
