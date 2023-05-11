/*
 * control_command_generator.cpp
 *
 *  Created on: Jan 8, 2018
 *      Author: qiao
 */
#include "dalu_robot/control_command_generator.h"

ControlCommandGenerator& ControlCommandGenerator::GetInstance()
{
  static ControlCommandGenerator generator;
  return generator;
}

ControlCommandGenerator::ControlCommandGenerator()
{
}

void ControlCommandGenerator::StopMoving(unsigned char* buffer)
{
  FormDiffDriveBaseCommand(buffer, 0.0f, 0.0f, 0.0f);
}

void ControlCommandGenerator::MoveForward(unsigned char* buffer, float line_speed)
{
  if (fabs(line_speed) > kMaxLineSpeed)
  {
    line_speed = kMaxLineSpeed;
  }
  FormDiffDriveBaseCommand(buffer, line_speed, 0.0f, 0.0f);
}

void ControlCommandGenerator::MoveBackward(unsigned char* buffer, float line_speed)
{
  if (fabs(line_speed) > kMaxLineSpeed)
  {
    line_speed = kMaxLineSpeed;
  }
  FormDiffDriveBaseCommand(buffer, -line_speed, 0.0f, 0.0f);
}

void ControlCommandGenerator::TurnRight(unsigned char* buffer, float angular_speed)
{
  if (fabs(angular_speed) > kMaxAngularSpeed)
  {
    angular_speed = kMaxAngularSpeed;
  }
  FormDiffDriveBaseCommand(buffer, 0.0f, 0.0f, -angular_speed);
}

void ControlCommandGenerator::TurnLeft(unsigned char* buffer, float angular_speed)
{
  if (fabs(angular_speed) > kMaxAngularSpeed)
  {
    angular_speed = kMaxAngularSpeed;
  }
  FormDiffDriveBaseCommand(buffer, 0.0f, 0.0f, angular_speed);
}

void ControlCommandGenerator::RaiseNeck(unsigned char* buffer)
{
  FormNeckControlCommand(buffer, kMaxNeckHeight);
}

void ControlCommandGenerator::LowerNeck(unsigned char* buffer)
{
  FormNeckControlCommand(buffer, kMinNeckHeight);
}

void ControlCommandGenerator::HandleHead(unsigned char* buffer, float yaw, float roll, float pitch)
{
  if (yaw > kMaxYawAngle)
  {
    yaw = kMaxYawAngle;
  }
  if (yaw < kMinYawAngle)
  {
    yaw = kMinYawAngle;
  }
  if (pitch > kMaxPitchAngle)
  {
    pitch = kMaxPitchAngle;
  }
  if (pitch < kMinYawAngle)
  {
    pitch = kMinYawAngle;
  }
  FormHeadControlCommand(buffer, yaw, 0, pitch);
}

void ControlCommandGenerator::YawHead(unsigned char* buffer, float angle)
{
  if (angle > kMaxYawAngle)
    angle = kMaxYawAngle;
  if (angle < kMinYawAngle)
    angle = kMinYawAngle;
  FormHeadControlCommand(buffer, angle, 0, 0);
}

void ControlCommandGenerator::PitchHead(unsigned char* buffer, float angle)
{
  if (angle > kMaxPitchAngle)
  {
    angle = kMaxPitchAngle;
  }
  if (angle < kMinYawAngle)
  {
    angle = kMinYawAngle;
  }
  FormHeadControlCommand(buffer, 0, 0, angle);
}

void ControlCommandGenerator::ResetHead(unsigned char* buffer)
{
  FormHeadControlCommand(buffer, 0, 0, 0);
}

void ControlCommandGenerator::RaiseBody(unsigned char* buffer, float height)
{
  if (fabs(height) < kMaxBodyHeight)
  {
    FormBodyControlCommand(buffer, height);
  }
  else
  {
    FormBodyControlCommand(buffer, kMaxBodyHeight);
  }
}

void ControlCommandGenerator::LowerBody(unsigned char* buffer, float height)
{
  if (fabs(height) < kMaxBodyHeight)
  {
    FormBodyControlCommand(buffer, height);
  }
  else
  {
    FormBodyControlCommand(buffer, kMaxBodyHeight);
  }
}

void ControlCommandGenerator::FormDiffDriveBaseCommand(unsigned char* buffer, float liner_speed_x, float liner_speed_y,
                                                       float angular_speed, int command_length)
{
  memset(buffer, '\0', command_length);
  buffer[0] = 0xaa;
  buffer[1] = 0x55;
  buffer[2] = 0x0f;
  buffer[3] = 0x12;
  buffer[4] = 0x01;
  buffer[5] = 0x00;

  float* liner_speed_x_ = (float*)(buffer + 6);
  float* liner_speed_y_ = (float*)(buffer + 10);
  float* angular_speed_ = (float*)(buffer + 14);

  *liner_speed_x_ = liner_speed_x;
  *liner_speed_y_ = liner_speed_y;

  //XXX:右手坐标系 适配底盘角速度取反
  *angular_speed_ = -angular_speed;
  unsigned char* sum = (unsigned char*)(buffer + (command_length - 1));
  *sum = FormSum(buffer, command_length);
#ifdef _DEBUG
  for (int i = 0; i < command_length; i++)
  {
    std::cout << std::hex << (int)buffer[i] << " ";
  }
  std::cout << std::endl;
#endif
}

void ControlCommandGenerator::FormDiffDriveBaseCommand(unsigned char* buffer, float liner_speed, float angular_speed,
                                                       int command_length)
{
  memset(buffer, '\0', command_length);
  buffer[0] = 0xaa;
  buffer[1] = 0x55;
  buffer[2] = 0x0b;
  buffer[3] = 0x12;
  buffer[4] = 0x01;
  buffer[5] = 0x00;

  float* linerSpeed_ = (float*)(buffer + 6);
  float* angularSpeed_ = (float*)(buffer + 10);

  *linerSpeed_ = liner_speed;
  // XXX(Qiao:)适配底盘角速度取反
  *angularSpeed_ = -angular_speed;
  unsigned char* sum = (unsigned char*)(buffer + (command_length - 1));
  *sum = FormSum(buffer, command_length);
#ifdef _DEBUG
  for (int i = 0; i < command_length; i++)
  {
    std::cout << std::hex << (int)buffer[i] << " ";
  }
  std::cout << std::endl;
#endif
}

void ControlCommandGenerator::FormBodyControlCommand(unsigned char* buffer, float height, int command_length)
{
  memset(buffer, '\0', command_length);
  buffer[0] = 0xaa;
  buffer[1] = 0x55;
  buffer[2] = 0x0f;
  buffer[3] = 0x13;
  buffer[4] = 0x01;
  buffer[5] = height;
  unsigned char* sum = (unsigned char*)(buffer + (command_length - 1));
  *sum = FormSum(buffer, command_length);
}

void ControlCommandGenerator::FormHeadControlCommand(unsigned char* buffer, float yaw, float roll, float pitch,
                                                     int command_length)
{
  memset(buffer, '\0', command_length);
  buffer[0] = 0xaa;
  buffer[1] = 0x55;
  buffer[2] = 0x0f;
  buffer[3] = 0x13;
  buffer[4] = 0x03;
  float* yaw_ = (float*)(buffer + 6);
  float* roll_ = (float*)(buffer + 10);
  float* pitch_ = (float*)(buffer + 14);

  *yaw_ = yaw;
  *roll_ = roll;
  *pitch_ = pitch;

  unsigned char* sum = (unsigned char*)(buffer + (command_length - 1));
  *sum = FormSum(buffer, command_length);
}

void ControlCommandGenerator::FormNeckControlCommand(unsigned char* buffer, float height, int command_length)
{
  memset(buffer, '\0', command_length);
  buffer[0] = 0xaa;
  buffer[1] = 0x55;
  buffer[2] = 0x0f;
  buffer[3] = 0x13;
  buffer[4] = 0x02;
  buffer[5] = (unsigned char)height;
  unsigned char* sum = (unsigned char*)(buffer + (command_length - 1));
  *sum = FormSum(buffer, command_length);
}

inline unsigned char ControlCommandGenerator::FormSum(unsigned char* buf, int len)
{
  unsigned char sum = 0x00;
  for (int pos = 0; pos < len - 1; pos++)
  {
    sum ^= buf[pos];
  }
  return sum;
}

bool ControlCommandGenerator::SumCheck(unsigned char* buf, int len)
{
  unsigned char i;
  unsigned char checksum = 0;

#ifdef _DEBUG
  for (i = 0; i < len; i++)
  {
    std::cout << std::hex << (int)buf[i] << " ";
  }
  std::cout << std::endl;
#endif

  for (i = 0; i < len - 1; i++)
  {
    checksum ^= buf[i];
  }

#ifdef _DEBUG
  std::cout << std::hex << (int)buf[len - 1] << " | " << (int)checksum << std::endl;
#endif

  return checksum == buf[len - 1] ? true : false;
}
