/*********************************************************************
 Refer to FORT SRC Manual for description
 Refer to Deep Orange used CAN messages for description
 *********************************************************************/

#ifndef _DO_CAN_DISPATCH_H
#define _DO_CAN_DISPATCH_H
#include <stdint.h>

namespace deeporange_dbw_ros
{

#undef BUILD_ASSERT

enum {
  // Fort Msgs
  ID_LEFT_JOY_BASIC                = 0xCFDD633,
  ID_LEFT_JOY_EXT                  = 0xCFDD934,
  ID_RIGHT_JOY_BASIC               = 0xCFDD834,
  ID_RIGHT_JOY_EXT                 = 0xCFDD733,
  ID_HEARTBEAT                     = 0xCFDE801,
  ID_REMOTE_STATUS                 = 0xCFDE861,
  ID_USER_VAL                      = 0xCFDE900,
  ID_USER_STRING                   = 0xCFDEA00,

  // Raptor msgs
  ID_VSS_MSG                       = 0x385,

  // ROS msgs
  ID_ROS_SUP_MSG                   = 0x386,
  ID_ROS_CONTROL_MSG               = 0x387,
  ID_ROS_CONTROL_TORQUE_MSG        = 0x388,  
  ID_ROS_CONTROL_VELOCITY_MSG      = 0x389,

  // Motor Encoder msgs
  ID_MOTOR1_POSITION               = 0X69,
  ID_MOTOR2_POSITION               = 0XCD,
  ID_MOTOR3_POSITION               = 0X131,
  ID_MOTOR4_POSITION               = 0X195,

};

} //deeporange_dbw_ros

#endif // _DO_CAN_DISPATCH_H
