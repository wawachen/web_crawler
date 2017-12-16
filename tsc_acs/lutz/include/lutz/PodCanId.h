/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

namespace tsc_acs {
namespace lutz {

enum PodCanId
{
  // Transmitted Messages
  ACS_AUX_POWER_REQ               = 0x107,
  ACS_CTRL_CMD1                   = 0x001,
  ACS_CTRL_CMD2                   = 0x2,
  ACS_HANDSHAKE                   = 0x211,
  DEBUG_EMU_CTRL_REQ              = 0x7ff,
  // Received Messages
  POD_AUX_POWER_STATUS            = 0x117,
  POD_BATTERY_STATUS              = 0x210,
  POD_DEBUG                       = 0x333,
  POD_DEBUG2                      = 0x334,
  POD_DI_STATE                    = 0x80,
  POD_EPB_STATUS                  = 0x110,
  POD_EPB_SW_VER                  = 0x705,
  POD_EPI_SW_VER                  = 0x702,
  POD_EPO_SW_VER                  = 0x703,
  POD_EPS_INBOARD_STATUS          = 0x064,
  POD_EPS_OUTBOARD_STATUS         = 0x065,
  POD_EPS_STATUS                  = 0x203,
  POD_FAULT_CODES                 = 0x220,
  POD_FRONT_ULTRASONIC_STATUS     = 0x105,
  POD_GWY_SW_VER                  = 0x704,
  POD_HANDSHAKE                   = 0x111,
  POD_IMU1_STATUS                 = 0x130,
  POD_IMU2_STATUS                 = 0x132,
  POD_POWERTRAIN_STATUS           = 0x200,
  POD_REAR_ULTRASONIC_STATUS      = 0x106,
  POD_STATUS                      = 0x108,
  POD_VMS_SW_VER                  = 0x700,
  POD_WDG_SW_VER                  = 0x701,
  POD_WHL_CNT                     = 0x102,
  POD_WHL_SW_VER                  = 0x706
};

}
}
