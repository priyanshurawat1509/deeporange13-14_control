/*********************************************************************
Refer to DO13 Raptor DBW state machine for info on states
 *********************************************************************/

#ifndef _DO_STATE_ENUM_H
#define _DO_STATE_ENUM_H
#include <stdint.h>

namespace deeporange_dbw_ros
{

#undef BUILD_ASSERT

enum {
  // ROS States
  AU_NOT_OK                = 0,
  AU_OK                    = 1,

  // System States
  SS1_DEFAULT               = 1,
  SS6_READY                 = 6,
  SS8_NOMINALOP             = 8,
  SS99_SAFESTOP             = 99,
  SS98_SAFESTOP_EXECUTED    = 98,
  SS31_SHUTDOWN_REQUESTED   = 31,
  SS32_SHUTDOWN             = 32,

  // ROS health/fault codes
  HEALTH_NOT_OK             = 0,
  HEALTH_OK                 = 1,
  FAULT1_TIMESYNC           = 1,
  FAULT2_xxx                = 2

};

} //deeporange_dbw_ros

#endif // _DO_STATE_ENUM_H
