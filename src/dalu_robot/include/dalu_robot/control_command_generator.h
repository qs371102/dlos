/*
 * control_command_generator.h
 *
 *  Created on: Jan 8, 2018
 *      Author: qiao
 */

#ifndef SRC_DALU_RTC_SRC_CONTROL_COMMAND_GENERATOR_H_
#define SRC_DALU_RTC_SRC_CONTROL_COMMAND_GENERATOR_H_

#include "dalu_robot/defaults.h"
#include <cstring>
#include <iostream>
#include <math.h>
#include <stdlib.h>

using namespace std;
using namespace dalu;

class ControlCommandGenerator
{
public:
    static ControlCommandGenerator& GetInstance();
    void StopMoving(unsigned char* buffer);
    void MoveForward(unsigned char* buffer, float line_speed = kMaxLineSpeed);
    void MoveBackward(unsigned char* buffer, float line_speed = kMaxLineSpeed);
    void TurnRight(unsigned char* buffer, float angular_speed = kMaxAngularSpeed);
    void TurnLeft(unsigned char* buffer, float angular_speed = kMaxAngularSpeed);
    void RaiseNeck(unsigned char* buffer);
    void LowerNeck(unsigned char* buffer);
    void HandleHead(unsigned char* buffer, float yaw, float roll, float pitch);
    void YawHead(unsigned char* buffer, float angle);
    void PitchHead(unsigned char* buffer, float angle);
    void ResetHead(unsigned char* buffer);
    void RaiseBody(unsigned char* buffer, float height = kMaxBodyHeight);
    void LowerBody(unsigned char* buffer, float height = kMinBodyHeight);
    void FormDiffDriveBaseCommand(unsigned char* buffer,
                                  float          line_speed,
                                  float          angular_speed,
                                  int            command_length = BASE_CONTROL_PACKAGE_LENGTH);

    void FormDiffDriveBaseCommand(unsigned char* buffer,
                                  float          liner_speed_x,
                                  float          liner_speed_y,
                                  float          angular_speed,
                                  int            command_length = BASE_CONTROL_PACKAGE_LENGTH);

    void FormBodyControlCommand(unsigned char* buffer,
                                float          height,
                                int            command_length = BODY_CONTROL_PACKAGE_LENGTH);

    void FormHeadControlCommand(unsigned char* buffer,
                                float          yaw,
                                float          roll,
                                float          pitch,
                                int            command_length = HEAD_AND_NECK_CONTROL_PACKAGE_LENGTH);

    void FormNeckControlCommand(unsigned char* buffer,
                                float          height,
                                int            command_length = HEAD_AND_NECK_CONTROL_PACKAGE_LENGTH);

    inline unsigned char FormSum(unsigned char* buf, int len);
    bool SumCheck(unsigned char* buf, int len);

    ControlCommandGenerator(ControlCommandGenerator const&) = delete; // copy ctor hidden
    ControlCommandGenerator& operator=(ControlCommandGenerator const&) = delete;

private:
    ControlCommandGenerator();
};

#endif /* SRC_DALU_RTC_SRC_CONTROL_COMMAND_GENERATOR_H_ */
