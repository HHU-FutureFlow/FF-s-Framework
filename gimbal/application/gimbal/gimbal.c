#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "can_comm.h"

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *pitch_motor;
static CANCommInstance *yaw_can_comm;

//板间通信内容的缓存
static Yaw_Ctrl_Cmd_s yaw_cmd_send;
static Yaw_Upload_Data_s yaw_feedback_recv;
static uint16_t yaw_motor_single_round_angle;


static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static BMI088Instance *bmi088; // 云台IMU

static float LimitPitchAngle(float pitch_ref)
{
    if (pitch_ref > PITCH_MAX_ANGLE)
        return PITCH_MAX_ANGLE;
    if (pitch_ref < PITCH_MIN_ANGLE)
        return PITCH_MIN_ANGLE;
    return pitch_ref;
}



void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 4,
                .Ki = 0,
                .Kd = 0.5,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 60, 
                .Ki = 0, 
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
   
    pitch_motor = DJIMotorInit(&pitch_config);


    CANComm_Init_Config_s yaw_comm_config = 
    {
    .can_config = 
    {
        .can_handle = &hcan1,
        .tx_id = 0x350,
        .rx_id = 0x351,
    },
    .send_data_len = sizeof(Yaw_Ctrl_Cmd_s),
    .recv_data_len = sizeof(Yaw_Upload_Data_s),
    .daemon_count = 10,
    };

    yaw_can_comm = CANCommInit(&yaw_comm_config);


    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    gimbal_cmd_recv.pitch = LimitPitchAngle(gimbal_cmd_recv.pitch);
    yaw_cmd_send.yaw_ref = gimbal_cmd_recv.yaw;
    yaw_cmd_send.yaw_angle = GYRO2GIMBAL_DIR_YAW * gimba_IMU_data->YawTotalAngle;
    yaw_cmd_send.yaw_gyro = GYRO2GIMBAL_DIR_YAW * gimba_IMU_data->Gyro[2];
    yaw_cmd_send.mode = (uint8_t)gimbal_cmd_recv.gimbal_mode;
    yaw_cmd_send.enable =gimbal_cmd_recv.gimbal_mode != GIMBAL_ZERO_FORCE;

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
       
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }


    CANCommSend(yaw_can_comm, (uint8_t *)&yaw_cmd_send);

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...


    if (CANCommIsOnline(yaw_can_comm))
    {
        Yaw_Upload_Data_s *yaw_feedback = (Yaw_Upload_Data_s *)CANCommGet(yaw_can_comm);

        yaw_feedback_recv = *yaw_feedback;
        yaw_motor_single_round_angle = yaw_feedback_recv.yaw_motor_single_round_angle;
    }


    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor_single_round_angle;
    gimbal_feedback_data.pitch_motor_single_round_angle = pitch_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
