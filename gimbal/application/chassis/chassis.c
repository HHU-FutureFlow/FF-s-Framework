#include "chassis.h"
#include "robot_def.h"
#include "power_control.h"
#include "n630.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"

#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)

#if CHASSIS_USE_N630_VESC
static N630MotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb;
#else
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb;
#endif

#ifdef CHASSIS_BOARD
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm;
attitude_t *Chassis_IMU_data;
#endif

#ifdef ONE_BOARD
static Publisher_t *chassis_pub;
static Subscriber_t *chassis_sub;
#endif

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;
static Chassis_Upload_Data_s chassis_feedback_data;
static PIDInstance buffer_PID;
static referee_info_t *referee_data;
static Referee_Interactive_info_t ui_data;
static SuperCapInstance *cap;
static float chassis_vx, chassis_vy;
static float vt_lf, vt_rf, vt_lb, vt_rb;
static float target_lf, now_lf, output_lf;

static float OutputShaftSpeedToMotorRef(float output_shaft_speed)
{
#if CHASSIS_USE_N630_VESC
    return output_shaft_speed * REDUCTION_RATIO_WHEEL * RAD_2_DEGREE;
#else
    return output_shaft_speed * REDUCTION_RATIO_WHEEL / RPM_2_RAD_PER_SEC;
#endif
}

static float MotorSpeedToOutputShaftSpeed(float motor_speed_deg_per_sec)
{
    return motor_speed_deg_per_sec * DEGREE_2_RAD / REDUCTION_RATIO_WHEEL;
}

void ChassisInit()
{
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 5.0f,
                .Ki = 1.5f,
                .Kd = 0.0f,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
                .Output_LPF_RC = 0.3f,
            },
        },
        .controller_setting_init_config = {
#if CHASSIS_USE_N630_VESC
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
#else
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
#endif
        },
#if CHASSIS_USE_N630_VESC
        .motor_type = N630,
#else
        .motor_type = M3508,
#endif
    };

    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;

#if CHASSIS_USE_N630_VESC
    motor_lf = N630MotorInit(&chassis_motor_config);
#else
    motor_lf = PowerControlInit(&chassis_motor_config);

#endif

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
#if CHASSIS_USE_N630_VESC
    motor_rf = N630MotorInit(&chassis_motor_config);
#else
    motor_rf = PowerControlInit(&chassis_motor_config);
#endif

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
#if CHASSIS_USE_N630_VESC
    motor_lb = N630MotorInit(&chassis_motor_config);
#else
    motor_lb = PowerControlInit(&chassis_motor_config);
#endif

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
#if CHASSIS_USE_N630_VESC
    motor_rb = N630MotorInit(&chassis_motor_config);
#else
    motor_rb = PowerControlInit(&chassis_motor_config);
#endif

    referee_data = UITaskInit(&huart6, &ui_data);

    PID_Init_Config_s Buffer_pid_conf = {
        .Kp = 0.1f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .IntegralLimit = 1000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 1000,
    };
    PIDInit(&buffer_PID, &Buffer_pid_conf);

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x302,
            .rx_id = 0x301,
        },
    };
    cap = SuperCapInit(&cap_conf);

#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init();
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf);
#endif

#ifdef ONE_BOARD
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

static void MecanumCalculate()
{
    vt_lf = (-chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER) / RADIUS_WHEEL;
    vt_rf = (-chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER) / RADIUS_WHEEL;
    vt_lb = (chassis_vx - chassis_vy - chassis_cmd_recv.wz * LB_CENTER) / RADIUS_WHEEL;
    vt_rb = (chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER) / RADIUS_WHEEL;
}


static void OmniCalculate()
{
    vt_lf = (-sqrt(2.0f) / 2.0f * chassis_vx - sqrt(2.0f) / 2.0f * chassis_vy + chassis_cmd_recv.wz * DEGREE_2_RAD * WHEEL_DIST_TO_CENTER) / RADIUS_WHEEL;
    vt_rf = (-sqrt(2.0f) / 2.0f * chassis_vx + sqrt(2.0f) / 2.0f * chassis_vy + chassis_cmd_recv.wz * DEGREE_2_RAD * WHEEL_DIST_TO_CENTER) / RADIUS_WHEEL;
    vt_lb = (sqrt(2.0f) / 2.0f * chassis_vx - sqrt(2.0f) / 2.0f * chassis_vy + chassis_cmd_recv.wz * DEGREE_2_RAD * WHEEL_DIST_TO_CENTER) / RADIUS_WHEEL;
    vt_rb = (sqrt(2.0f) / 2.0f * chassis_vx + sqrt(2.0f) / 2.0f * chassis_vy + chassis_cmd_recv.wz * DEGREE_2_RAD * WHEEL_DIST_TO_CENTER) / RADIUS_WHEEL;
}


static void LimitChassisOutput()
{
#if CHASSIS_USE_N630_VESC
    N630MotorSetRef(motor_lf, OutputShaftSpeedToMotorRef(vt_lf));
    N630MotorSetRef(motor_rf, OutputShaftSpeedToMotorRef(vt_rf));
    N630MotorSetRef(motor_lb, OutputShaftSpeedToMotorRef(vt_lb));
    N630MotorSetRef(motor_rb, OutputShaftSpeedToMotorRef(vt_rb));
#else
    DJIMotorSetRef(motor_lf, OutputShaftSpeedToMotorRef(vt_lf));
    DJIMotorSetRef(motor_rf, OutputShaftSpeedToMotorRef(vt_rf));
    DJIMotorSetRef(motor_lb, OutputShaftSpeedToMotorRef(vt_lb));
    DJIMotorSetRef(motor_rb, OutputShaftSpeedToMotorRef(vt_rb));
#endif
}

static void EstimateSpeed()
{
}

void ChassisTask()
{
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif

    SetPowerLimit(1000000.0f);
    //SetPowerLimit(referee_data->GameRobotState.chassis_power_limit);

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    {

#if CHASSIS_USE_N630_VESC
        N630MotorEnable(motor_lf);
        N630MotorEnable(motor_rf);
        N630MotorEnable(motor_lb);
        N630MotorEnable(motor_rb);
#else
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
#endif
    }

    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW:
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW:
        chassis_cmd_recv.wz = -1.5f * chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);
        break;
    case CHASSIS_ROTATE:
        chassis_cmd_recv.wz = 60;
        break;
    default:
        break;
    }

    // static float sin_theta, cos_theta;
    // cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    // sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    // chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    // chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
    chassis_vx = chassis_cmd_recv.vx;
    chassis_vy = chassis_cmd_recv.vy ;

#if CHASSIS_TYPE
    MecanumCalculate();
#else
    OmniCalculate();
#endif

    target_lf = vt_lf;
    now_lf = MotorSpeedToOutputShaftSpeed(motor_lf->measure.speed_aps);
    LimitChassisOutput();
    EstimateSpeed();
    output_lf = motor_lf->motor_controller.speed_PID.Output;

#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif
}
