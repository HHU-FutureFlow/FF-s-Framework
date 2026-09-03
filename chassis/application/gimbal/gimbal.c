#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "can_comm.h"

static DJIMotorInstance *yaw_motor;
static CANCommInstance *yaw_can_comm;

static Yaw_Ctrl_Cmd_s yaw_cmd_recv;
static Yaw_Upload_Data_s yaw_feedback_send;
static float yaw_speed_feedback_deg_per_sec;

/* Yaw 参考值斜坡限速参数,防止 90° 这类大阶跃直接饱和电流产生冲击异响.
 * 需结合实机运动能力标定,这里的数值为较保守的初始值. */
#define YAW_REF_MAX_SPEED 150.0f  // 参考最大变化速度,单位 deg/s
#define YAW_REF_MAX_ACCEL 600.0f  // 参考最大加速度,单位 deg/s^2
#define YAW_SETTLE_ERR_DEG 0.5f   // 判断 Yaw 是否已进入稳定区, 单位 deg

static float yaw_ref_ramp_out;
static float yaw_ref_ramp_speed;
static uint8_t yaw_ref_ramp_init;

/* 重置斜坡状态, 停止/离线/零力矩模式下恢复初始化, 下次启用时从当前目标开始 */
static void RampYawRefReset(void)
{
    yaw_ref_ramp_init = 0;
    yaw_ref_ramp_speed = 0.0f;
}

static float RampYawRef(float target)
{
    const float dt = 0.005f; // GimbalTask 运行于 200Hz

    if (!yaw_ref_ramp_init)
    {
        yaw_ref_ramp_out = target;
        yaw_ref_ramp_speed = 0.0f;
        yaw_ref_ramp_init = 1;
        return yaw_ref_ramp_out;
    }

    /* 限速 */
    float target_speed = (target - yaw_ref_ramp_out) / dt;
    if (target_speed > YAW_REF_MAX_SPEED)
        target_speed = YAW_REF_MAX_SPEED;
    if (target_speed < -YAW_REF_MAX_SPEED)
        target_speed = -YAW_REF_MAX_SPEED;

    /* 限加速度 */
    float speed_step = target_speed - yaw_ref_ramp_speed;
    float max_speed_step = YAW_REF_MAX_ACCEL * dt;
    if (speed_step > max_speed_step)
        speed_step = max_speed_step;
    if (speed_step < -max_speed_step)
        speed_step = -max_speed_step;

    yaw_ref_ramp_speed += speed_step;
    yaw_ref_ramp_out += yaw_ref_ramp_speed * dt;
    return yaw_ref_ramp_out;
}

// Standalone debug symbols for VarScope.这里是为了调参临时创建的调试变量，方便Varscope监视。
volatile float yaw_debug_angle_pid_ref;
volatile float yaw_debug_angle_pid_measure;
volatile float yaw_debug_angle_pid_err;
volatile float yaw_debug_angle_pid_pout;
volatile float yaw_debug_angle_pid_iout;
volatile float yaw_debug_angle_pid_dout;
volatile float yaw_debug_angle_pid_output;
volatile float yaw_debug_speed_pid_ref;
volatile float yaw_debug_speed_pid_measure;
volatile float yaw_debug_speed_pid_err;
volatile float yaw_debug_speed_pid_pout;
volatile float yaw_debug_speed_pid_iout;
volatile float yaw_debug_speed_pid_dout;
volatile float yaw_debug_speed_pid_output;
volatile float yaw_debug_motor_speed_aps;
volatile float yaw_debug_motor_current;
volatile float yaw_debug_motor_angle_single_round;
volatile float yaw_debug_motor_total_angle;
volatile float yaw_debug_gyro_deg_per_sec;

static void UpdateYawDebugData()
{
    if (yaw_motor == NULL)
        return;

    yaw_debug_angle_pid_ref = yaw_motor->motor_controller.angle_PID.Ref;
    yaw_debug_angle_pid_measure = yaw_motor->motor_controller.angle_PID.Measure;
    yaw_debug_angle_pid_err = yaw_motor->motor_controller.angle_PID.Err;
    yaw_debug_angle_pid_pout = yaw_motor->motor_controller.angle_PID.Pout;
    yaw_debug_angle_pid_iout = yaw_motor->motor_controller.angle_PID.Iout;
    yaw_debug_angle_pid_dout = yaw_motor->motor_controller.angle_PID.Dout;
    yaw_debug_angle_pid_output = yaw_motor->motor_controller.angle_PID.Output;

    yaw_debug_speed_pid_ref = yaw_motor->motor_controller.speed_PID.Ref;
    yaw_debug_speed_pid_measure = yaw_motor->motor_controller.speed_PID.Measure;
    yaw_debug_speed_pid_err = yaw_motor->motor_controller.speed_PID.Err;
    yaw_debug_speed_pid_pout = yaw_motor->motor_controller.speed_PID.Pout;
    yaw_debug_speed_pid_iout = yaw_motor->motor_controller.speed_PID.Iout;
    yaw_debug_speed_pid_dout = yaw_motor->motor_controller.speed_PID.Dout;
    yaw_debug_speed_pid_output = yaw_motor->motor_controller.speed_PID.Output;

    yaw_debug_motor_speed_aps = yaw_motor->measure.speed_aps;
    yaw_debug_motor_current = yaw_motor->measure.real_current;
    yaw_debug_motor_angle_single_round = yaw_motor->measure.angle_single_round;
    yaw_debug_motor_total_angle = yaw_motor->measure.total_angle;
    yaw_debug_gyro_deg_per_sec = yaw_speed_feedback_deg_per_sec;
}

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
float yaw_rpm; // 云台yaw电机转速,单位rpm

static BMI088Instance *bmi088; // 云台IMU
void GimbalInit()
{   
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 5,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 6, // 8
                .Ki = 0,
                .Kd = 0.5,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,

                .MaxOut = 150, // 500->150,降低速度参考上限,避免阶跃时电机满电流冲击
            },
            .speed_PID = {
                .Kp = 30, // 60->30,配合 CAN 链路反馈延迟降低增益
                .Ki = 5,  // 20->5,削弱积分堆积,防止悬停时积分残留过大
                .Kd = 0,
                .DeadBand = 1.0, // 新增:悬停时进入死区,避免持续输出力矩
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 1000, // 3000->1000
                .MaxOut = 10000,       // 20000->10000,电流封顶,避免撞到 GM6020 协议上限 16384
            },
            .other_angle_feedback_ptr = &yaw_cmd_recv.yaw_angle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &yaw_speed_feedback_deg_per_sec,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            
        },
        .motor_type = GM6020};
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    CANComm_Init_Config_s yaw_comm_config = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x351,
            .rx_id = 0x350,
        },
        .send_data_len = sizeof(Yaw_Upload_Data_s),
        .recv_data_len = sizeof(Yaw_Ctrl_Cmd_s),
        .daemon_count = 10,
    };

    yaw_can_comm = CANCommInit(&yaw_comm_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    if (!CANCommIsOnline(yaw_can_comm))
    {
        DJIMotorStop(yaw_motor);
        /* 离线/停止时清零两级 PID 积分, 避免恢复时积分残留导致异响 */
        yaw_motor->motor_controller.angle_PID.Iout = 0.0f;
        yaw_motor->motor_controller.speed_PID.Iout = 0.0f;
        RampYawRefReset();

        yaw_feedback_send.yaw_motor_single_round_angle =
            yaw_motor->measure.angle_single_round;
        yaw_feedback_send.yaw_angle = yaw_cmd_recv.yaw_angle;
        yaw_feedback_send.yaw_speed = yaw_motor->measure.speed_aps;
        yaw_feedback_send.online = 0;

        UpdateYawDebugData();
        CANCommSend(yaw_can_comm, (uint8_t *)&yaw_feedback_send);
        return;
    }

    yaw_cmd_recv = *(Yaw_Ctrl_Cmd_s *)CANCommGet(yaw_can_comm);
    yaw_speed_feedback_deg_per_sec = yaw_cmd_recv.yaw_gyro * RAD_2_DEGREE;

    if (!yaw_cmd_recv.enable ||
        yaw_cmd_recv.mode == GIMBAL_ZERO_FORCE)
    {
        DJIMotorStop(yaw_motor);
        /* 停止/零力矩时清零积分并重置斜坡, 避免重新启用时冲击 */
        yaw_motor->motor_controller.angle_PID.Iout = 0.0f;
        yaw_motor->motor_controller.speed_PID.Iout = 0.0f;
        RampYawRefReset();
    }
    else
    {
        DJIMotorEnable(yaw_motor);

        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);

        /* 目标输入先过斜坡限速/限加速, 防止 90° 阶跃直接撞硬限幅 */
        DJIMotorSetRef(yaw_motor, RampYawRef(yaw_cmd_recv.yaw_ref));

        /* 接近目标(悬停)时清零速度环积分, 避免积分残留导致持续力矩和嗡嗡声 */
        if (yaw_motor->motor_controller.angle_PID.Err < YAW_SETTLE_ERR_DEG &&
            yaw_motor->motor_controller.angle_PID.Err > -YAW_SETTLE_ERR_DEG)
        {
            yaw_motor->motor_controller.speed_PID.Iout = 0.0f;
        }
    }

    yaw_feedback_send.yaw_motor_single_round_angle =
        yaw_motor->measure.angle_single_round;
    yaw_feedback_send.yaw_angle = yaw_cmd_recv.yaw_angle;
    yaw_feedback_send.yaw_speed = yaw_motor->measure.speed_aps;
    yaw_feedback_send.online = 1;

    CANCommSend(yaw_can_comm, (uint8_t *)&yaw_feedback_send);
    yaw_rpm = (yaw_motor->measure.speed_aps)/6.0f;
    UpdateYawDebugData();
    
}
