/*
 * defaults.h
 *
 *  Created on: Nov 2, 2017
 *      Author: qiao
 */

#ifndef SRC_DALU_ROBOT_INCLUDE_DALU_ROBOT_DEFAULTS_H_
#define SRC_DALU_ROBOT_INCLUDE_DALU_ROBOT_DEFAULTS_H_
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <math.h>
#include <iostream>

namespace dalu
{
#define BASE_CONTROL_PACKAGE_LENGTH 19
#define BASE_CALLBACK_PACKAGE_LENGTH 28
#define HEAD_AND_NECK_CONTROL_PACKAGE_LENGTH 19
#define BODY_CONTROL_PACKAGE_LENGTH 19

const float kMaxLineSpeed = 0.25;
const float kMaxAngularSpeed = 0.8;
const int kMaxBodyHeight = 100;
const int kMinBodyHeight = 0;
const int kMaxNeckHeight = 100;
const int kMinNeckHeight = 0;
const float kMaxYawAngle = 60;
const float kMinYawAngle = -60;
const float kMaxPitchAngle = 20;
const float kMinPitchAngle = -15;

enum RobotStatus
{
  STATUS_IDLE,     //初始
  STATUS_PATROL,   //巡逻
  STATUS_MAPPING,  //建图
  STATUS_CONTROL,  //控制
  STATUS_CHARGING, //充电
  STATUS_BLOCKED,  //阻塞
  STATUS_NAV_TO_CHARGING_STATION //充电引导
};

enum State
{
  IDLE = 0, PATROL = 1,
};

enum Deriections
{
  NONE = -1, FRONT, RIGHT_FRONT, RIGHT, RIGHT_REAR, REAR, LEFT_REAR, LEFT, LEFT_FRONT,
};

inline void printCommands(unsigned char *cmds, int length)
{
  for (int i = 0; i < length; i++)
  {
    std::cout << std::hex << (int)cmds[i] << " ";
  }
  std::cout << std::endl;
}

inline Deriections calculateMoveDeriection(double line_speed, double angular_speed)
{
  if (fabs(angular_speed) == 0.0)
  {
    if (line_speed > 0.0)
    {
      return FRONT;
    }
    else if (line_speed < 0.0)
    {
      return REAR;
    }
  }
  else if (fabs(line_speed) == 0.0)
  {
    if (angular_speed > 0.0)
    {
      return RIGHT;
    }
    else if (angular_speed < 0.0)
    {
      return LEFT;
    }
  }
  else
  {
    if (line_speed > 0)
    {
      if (angular_speed > 0)
      {
        return RIGHT_FRONT;
      }
      else if (angular_speed < 0)
      {
        return LEFT_FRONT;
      }
    }
    else
    {
      if (angular_speed > 0)
      {
        return LEFT_REAR;
      }
      else if (angular_speed < 0)
      {
        return RIGHT_REAR;
      }
    }
  }
  return NONE;
}
}

#endif /* SRC_DALU_ROBOT_INCLUDE_DALU_ROBOT_DEFAULTS_H_ */
