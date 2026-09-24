#ifndef N630_H
#define N630_H

#include <stdint.h>

#include "bsp_can.h"
#include "controller.h"
#include "daemon.h"
#include "motor_def.h"

#define N630_MOTOR_CNT 12

#define N630_SPEED_SMOOTH_COEF 0.85f
#define N630_CURRENT_SMOOTH_COEF 0.90f

typedef struct
{
    uint16_t last_ecd;
    uint16_t ecd;
    float angle_single_round;
    float speed_aps;
    float real_current;
    uint8_t temperature;
    uint16_t fault_code;
    float total_angle;
    int32_t total_round;
    uint8_t online;
} N630_Motor_Measure_s;

typedef struct
{
    N630_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    Motor_Controller_s motor_controller;
    CANInstance *motor_can_instance;
    Motor_Type_e motor_type;
    Motor_Working_Type_e stop_flag;
    DaemonInstance *daemon;
    uint32_t feed_cnt;
    float dt;
} N630MotorInstance;

N630MotorInstance *N630MotorInit(Motor_Init_Config_s *config);
void N630MotorChangeFeed(N630MotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);
void N630MotorStop(N630MotorInstance *motor);
void N630MotorEnable(N630MotorInstance *motor);
void N630MotorOuterLoop(N630MotorInstance *motor, Closeloop_Type_e outer_loop);
void N630MotorSetRef(N630MotorInstance *motor, float ref);
void N630MotorControl(void);

#endif
