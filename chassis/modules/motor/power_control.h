#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"
#include "dji_motor.h"

DJIMotorInstance *PowerControlInit(Motor_Init_Config_s *config);
void PowerControl(void);
void SetPowerLimit(float power_limit);
float GetPowerLimit(void);

#endif
