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

#pragma once
#include <stdint.h>

#pragma pack(push, 1)

typedef struct {
  uint32_t gps_minutes;
  uint8_t num_sats;
  uint8_t position_mode;
  uint8_t velocity_mode;
  uint8_t orientation_mode;
} GnssInfo;

typedef struct {
  uint8_t x;    // x0.1sd
  uint8_t y;
  uint8_t z;
  uint8_t vel_x;
  uint8_t vel_y;
  uint8_t vel_z;
  uint8_t pitch;
  uint8_t heading;
} KalmanInnovation1;

typedef struct {
  uint16_t byte_count;
  uint16_t packet_count;
  uint16_t bad_bytes;
  uint16_t old_packets;
} ReceiverInfo;

typedef struct {
  int16_t north; // mm (or mm/s)
  int16_t east;
  int16_t down;
  uint8_t age;
  uint8_t status;
} Position; // (or Velocity )

typedef struct {
  int16_t heading;
  int16_t pitch;
  int16_t roll;
  uint8_t age;
} Orientation;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
  uint8_t age;
} XYZ;

typedef struct {
  int16_t heading;
  int16_t pitch;
  int16_t distance;
  uint8_t age;
} Polar;

typedef union {
  uint8_t bytes[8];
  GnssInfo gnssInfo;
  KalmanInnovation1 kalmanInnovation1;
  ReceiverInfo receiverInfo;
  Position position;
  Orientation orientation;
  XYZ xyz;
  Polar polar;
  /*
  struct {
    int16_t base_station_age;
    int8_t base_station_id[4];
  } chan20;
  struct {
    uint16_t delay_ms;
  } chan23;
  struct {
    uint8_t heading_quality; // 0:None 1:Poor 2:RTK Float 3:RTK Integer
  } chan27;
  struct {
    int16_t heading_misalignment_angle; // 1e-4 rad
    uint16_t heading_misalignment_accuracy; // 1e-4 rad
    uint16_t :16;
    uint8_t valid;
  } chan37;
  struct {
    int16_t undulation; // 5e-3 m
    uint8_t HDOP; // 1e-1
    uint8_t PDOP; // 1e-1
  } chan48;
  */
} Channel;

typedef struct {
  uint8_t sync;                         // 0
  uint16_t time;                        // 1
  int32_t accel_x :24; // 1e-5 m/s^2    // 3
  int32_t accel_y :24; // 1e-5 m/s^2    // 6
  int32_t accel_z :24; // 1e-5 m/s^2    // 9
  int32_t gyro_x :24; // 1e-4 rad/s     // 12
  int32_t gyro_y :24; // 1e-4 rad/s     // 15
  int32_t gyro_z :24; // 1e-4 rad/s     // 18
  uint8_t nav_status;                   // 21
  uint8_t chksum1;                      // 22
  double latitude;                      // 23
  double longitude;                     // 31
  float altitude;                       // 39
  int32_t vel_north :24; // 1e-4 m/s    // 43
  int32_t vel_east :24; // 1e-4 m/s     // 46
  int32_t vel_down :24; // 1e-4 m/s     // 49
  int32_t heading :24; // 1e-6 rad      // 52
  int32_t pitch :24; // 1e-6 rad        // 55
  int32_t roll :24; // 1e-6 rad         // 58
  uint8_t chksum2;                      // 61
  uint8_t channel;                      // 62
  Channel chan;                         // 63
  uint8_t chksum3;                      // 71
} Packet;

#pragma pack(pop)

enum {
  MODE_NONE = 0,
  MODE_SEARCH = 1,
  MODE_DOPLER = 2,
  MODE_SPS = 3,
  MODE_DIFFERENTIAL = 4,
  MODE_RTK_FLOAT = 5,
  MODE_RTK_INTEGER = 6,
  MODE_WAAS = 7,
  MODE_OMNISTAR_VBS = 8,
  MODE_OMNISTAR_HP = 9,
  MODE_NO_DATA = 10,
  MODE_BLANKED = 11,
  MODE_DOPLER_PP = 12,
  MODE_SPS_PP = 13,
  MODE_DIFFERENTIAL_PP = 14,
  MODE_RTK_FLOAT_PP = 15,
  MODE_RTK_INTEGER_PP = 16,
  MODE_OMNISTAR_XP = 17,
  MODE_CDGPS = 18,
  MODE_NOT_RECOGNISED = 19,
  MODE_UNKNOWN = 20,
};

enum {
  CH_GNSS_INFO = 0, // Full time, number of satellites, position mode, velocity mode, dual antenna mode. Table 8
  CH_KALMAN_INNOVATION_1 = 1, // Kalman filter innovations set 1 (position, velocity, attitude). Table 10
  CH_PRIMARY_RECEIVER_INFO = 2, // Internal information about primary GNSS receiver. Table 11
  CH_POSITION_ACCURACY = 3, // Position accuracy. Table 12
  CH_VELOCITY_ACCURACY = 4, // Velocity accuracy. Table 13
  CH_ORIENTATION_ACCURACY = 5, // Orientation accuracy. Table 15
  CH_GYRO_BIAS = 6, // Gyro bias. Table 16
  CH_ACCELEROMETER_BIAS = 7, // Accelerometer bias. Table 17
  CH_GRYO_SCALE = 8, // Gyro scale factor. Table 18
  CH_GYRO_BIAS_ACCURACY = 9, // Gyro bias accuracy. Table 19
  CH_ACCELEROMETER_BIAS_ACCURACY = 10, // Accelerometer bias accuracy. Table 20
  CH_GRYO_SCALE_ACCURACY = 11, // Gyro scale factor accuracy. Table 21
  CH_PRIMARY_ANTENNA_POSTION = 12, // Position estimate of the primary GNSS antenna. Table 22
  CH_ORIENTATION_DUAL = 13, // Orientation estimate of dual antenna systems. Table 23
  CH_PRIMARY_ANTENNA_POSITION_ACCURACY = 14, // Position accuracy of the primary GNSS antenna. Table 24
  CH_ORIENTATION_DUAL_ACCURACY = 15, // Orientation accuracy of the dual antenna systems. Table 25
  CH_INS_HOST_ROTATION = 16, // INS to host object rotation. Table 26
  CH_SECONDARY_RECEIVER_INFO = 17, // Internal information about secondary GNSS receiver. Table 27
  CH_IMU_INFO = 18, // Internal information about inertial measurement unit (IMU). Table 28
  CH_INS_VERSION = 19, // INS software version. Table 29
  CH_DIFFERENTIAL_CORRECTION_INFO = 20, // Differential correction information. Table 30
  CH_DISK_SPACE = 21, // Disk space, size of current internal log file. Table 31
  CH_REAL_TIME_TIMING = 22, // Internal information on timing of real-time processing. Table 32
  CH_UP_TIME = 23, // System up-time, number of consecutive GNSS rejections. Table 33
  CH_EVENT_TRIGGER_FALLING = 24, // Asynchronous packet triggered by falling edge of event input. Table 34
  CH_OUTPUT_DISPLACEMENT = 26, // Output displacement lever arm. Table 35
  CH_DUAL_AMBIGUITY_0 = 27, // Internal information about dual antenna ambiguity searches. Table 36
  CH_DUAL_AMBIGUITY_1 = 28, // Internal information about dual antenna ambiguity searches. Table 37
  CH_NAVCONFIG = 29, // Initial settings defined with NAVconfig. Table 38
  CH_OS_VERSION = 30, // Operating system and script version information. Table 39
  CH_HARDWARE_CONFIG = 31, // Hardware configuration information. –
  CH_KALMAN_INNOVATION_2 = 32, // Kalman filter innovations set 2. Table 40
  CH_ZERO_VELOCITY_LEVER = 33, // Zero velocity lever arm. Table 41
  CH_ZERO_VELOCITY_LEVEL_ACCURACY = 34, // Zero velocity lever arm accuracy. Table 42
  CH_LATERAL_SLIP_LEVER = 35, // Lateral advanced slip lever arm. Table 43
  CH_LATERAL_SLIP_LEVER_ACCURACY = 36, // Lateral advanced slip lever arm accuracy. Table 44
  CH_HEADING_MISALIGNMENT = 37, // Heading misalignment angle. Table 45
  CH_ZERO_VELOCITY_OPTION = 38, // Zero velocity option settings, third serial output mode –
  CH_LATERAL_SLIP_OPTION = 39, // Lateral advanced slip option settings. –
  CH_NCOM_VERSION = 40, // NCOM version ID. Table 46
  CH_OUTPUT_BAUD = 41, // Output baud rates. Table 47
  CH_HEADING_LOCK_OPTIONS = 42, // Heading lock options. Table 48
  CH_EVENT_TRIGGER_RISING = 43, // Asynchronous packet triggered by rising edge of event input. Table 34
  CH_WHEEL_SPEED_CONFIG = 44, // Wheel speed configuration. Table 51
  CH_WHEEL_SPEED_COUNTS = 45, // Wheel speed counts. Table 52
  CH_WHEEL_SPEED_LEVER = 46, // Wheel speed lever arm. Table 53
  CH_WHEEL_SPEED_LEVER_ACCURACY = 47, // Wheel speed lever arm accuracy. Table 54
  CH_DOP_GPS = 48, // Undulation, dilution of precision (DOP) of GPS. Table 55
  CH_OMNISTAR = 49, // OmniSTAR tracking information. Table 56
  CH_COMMAND_DECODER = 50, // Information sent to the command decoder. Table 57
  CH_SLIP_POINT_1 = 51, // Additional slip point 1 lever arm. Table 58
  CH_SLIP_POINT_2 = 52, // Additional slip point 2 lever arm. Table 58
  CH_SLIP_POINT_3 = 53, // Additional slip point 3 lever arm. Table 58
  CH_SLIP_POINT_4 = 54, // Additional slip point 4 lever arm. Table 58
  CH_PRIMARY_GNSS_INFO_1 = 55, // Information about the primary GNSS receiver. Table 59
  CH_SECONDARY_GNSS_INFO_1 = 56, // Information about the secondary GNSS receiver. Table 59
  CH_PRIMARY_GNSS_POSITION = 57, // Position estimate of the primary GNSS antenna (extended range). Table 60
  CH_VEHICLE_OUTPUT_FRAME_ROTATION = 58, // Vehicle to output frame rotation. Table 61
  CH_IMU_DECODE_STATUS = 59, // IMU decoding status. Table 62
  CH_SURFACE_ANGLES_DEF = 60, // Definition of the surface angles. Table 63
  CH_EXTERNAL_GNSS_INFO = 61, // Internal information about external GNSS receiver. Table 64
  CH_EXTERNAL_GNSS_INFO_1 = 62, // Information about the external GNSS receiver. Table 59
  CH_ANGULAR_ACCEL_SETTINGS = 63, // Angular acceleration filter settings. –
  CH_HARDWARE_INFO = 64, // Hardware information and external GNSS receiver configuration. Table 65
  CH_CAMERA_TRIGGER = 65, // Asynchronous packet triggered by camera/distance output. Table 34
  CH_LAT_LNG_DEF = 66, // Extended local co-ordinate definition, latitude and longitude. Table 66
  CH_ALT_HEAD_DEF = 67, // Extended local co-ordinate definition, altitude and heading. Table 67
  CH_SLIP_POINT_5 = 68, // Additional slip point 5 lever arm. Table 58
  CH_SLIP_POINT_6 = 69, // Additional slip point 6 lever arm. Table 58
  CH_SLIP_POINT_7 = 70, // Additional slip point 7 lever arm. Table 58
  CH_SLIP_POINT_8 = 71, // Additional slip point 8 lever arm. Table 58
  CH_STATUS_INFO_0 = 72, // Status information. Table 68
  CH_STATUS_INFO_1 = 73, // Status information. Table 69
  CH_LINEAR_ACCEL_SETTINGS = 74, // Linear acceleration filter settings. –
  CH_MISC = 75, // Miscellaneous. Table 70
  CH_DIFFERENTIAL_CORRECTION_INFO_INTERNAL = 76, // Internal information about differential corrections. Table 71
  CH_DIFFERENTIAL_CORRECTION_CONFIG = 77, // Differential correction configuration. Table 72
  CH_CAN_STATUS = 78, // CAN bus status information. Table 73
  CH_EVENT_2_TRIGGER_FALLING = 79, // Asynchronous packet triggered by falling edge of event input 2 (for xNAV only). Table 34
  CH_EVENT_2_TRIGGER_RISING = 80, // Asynchronous packet triggered by rising edge of event input 2 (for xNAV only). Table 34
  CH_CAMERA_2_TRIGGER = 81, // Asynchronous packet triggered by camera/distance output 2 (for xNAV only). Table 34
  CH_HARDWARE_CONFIG_2 = 82, // Hardware configuration information (for xNAV only). –
  CH_STATUS_INFO_2 = 83, // Status information (for xNAV only) –
  CH_STATUS_INFO_3 = 84, // Status information (for xNAV only) –
  CH_KALMAN_INNOVATION_3 = 88, // Kalman filter innovations set 3. Table 74
  CH_VERTICAL_SLIP_LEVER = 89, // Vertical advanced slip lever arm. Table 75
  CH_VERTICAL_SLIP_LEVER_ACCURACY = 90, // Vertical advanced slip lever arm accuracy. Table 76
  CH_PITCH_MISALIGNMENT = 91, // Pitch misalignment angle. Table 77
  CH_VERTICAL_SLIP_LEVER_OPTIONS = 92, // Vertical advanced slip option settings. –
};
