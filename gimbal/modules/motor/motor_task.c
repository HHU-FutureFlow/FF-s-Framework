#include "motor_task.h"
#include "robot_def.h"
#include "LK9025.h"
#include "HT04.h"
#include "dji_motor.h"
#include "n630.h"
#include "step_motor.h"
#include "servo_motor.h"
#include "power_control.h"

void MotorControlTask()
{
    DJIMotorControl();

#if CHASSIS_USE_N630_VESC
    N630MotorControl();
#else
    PowerControl();
#endif

    // LKMotorControl();
}
