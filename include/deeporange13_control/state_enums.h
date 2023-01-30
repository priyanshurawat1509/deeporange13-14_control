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
  AU0_DEFAULT               = 0,
  AU1_NOT_OK                = 1,
  AU2_NAV_ACTIVE            = 2,
  AU3_NAV_INACTIVE          = 3,
  AU254_SHUTDOWN            = 254,

  // System States
  SS1_DEFAULT               = 1,
  SS2_WAKE		     = 2,
  SS6_READY                 = 6,
  SS8_NOMINALOP             = 8,
  SS99_SAFESTOP             = 99,
  SS98_SAFESTOP_EXECUTED    = 98,
  SS31_SHUTDOWN_REQUESTED   = 31,
  SS32_SHUTDOWN             = 32,

  // ROS health/fault codes
  HEALTH0_DEFAULT              = 0,
  HEALTH1_FATAL               = 1,
  HEALTH2_WARN               = 2,
  HEALTH3_OK                 = 3,

  FAULT1_TIMESYNC           = 1,
  FAULT2_xxx                = 2,

  HEALTH_ASPECT_FOUND       = 2,
  HEALTH_ASPECT_NOT_FOUND   = 1,
  HEALTH_ASPECT_SEARCHING   = 0


};

} //deeporange_dbw_ros

#endif // _DO_STATE_ENUM_H
