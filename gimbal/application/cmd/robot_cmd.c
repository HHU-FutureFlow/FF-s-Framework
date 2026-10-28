// app
#include "robot_def.h"

// Temporary yaw step test. Set to 0 to remove this test behavior.
#define YAW_STEP_TEST_ENABLE 0 // 改动清单B: 关闭云台 yaw 扫描测试(自走), 恢复视觉正常交回控制权
#define YAW_STEP_TEST_ANGLE 90.0f

#if YAW_STEP_TEST_ENABLE
volatile float yaw_test_key_rise;
volatile float yaw_test_target;
static uint8_t yaw_test_last_key_count;
#endif

#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "sp_protocol.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
// module(遥控转发帧组帧用)
#include "crc8.h"
#include <string.h>
#include <stdlib.h>

// 裁判系统真实弹速反馈(m/s), 定义于 referee_task.c, 取值自裁判 0x0207 ShootData.bullet_speed
extern volatile float dbg_bullet_speed;

// VarScope 观测变量: 视觉SP下发 yaw 命令(deg) / 实际写入云台 yaw 目标(deg)
volatile float dbg_vision_yaw_cmd = 0.0f;
volatile float dbg_gimbal_yaw_ref = 0.0f;
// VarScope 观测变量: 视觉SP下发 pitch 原始值(rad) / 取反转deg后的命令(deg)
volatile float dbg_vision_pitch = 0.0f;
volatile float dbg_vision_pitch_cmd = 0.0f;

// VarScope 观测变量: 遥控器摇杆/拨轮原始值(桥接 rc_data[TEMP].rc.*, 用于验证中位残留)
volatile float dbg_rc_rocker_l_ = 0.0f; // 左水平
volatile float dbg_rc_rocker_l1 = 0.0f; // 左竖直
volatile float dbg_rc_rocker_r_ = 0.0f; // 右水平
volatile float dbg_rc_rocker_r1 = 0.0f; // 右竖直
volatile float dbg_rc_dial      = 0.0f; // 拨轮

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

static float LimitPitchTarget(float pitch_target)
{
    if (pitch_target > PITCH_MAX_ANGLE)
        return PITCH_MAX_ANGLE;
    if (pitch_target < PITCH_MIN_ANGLE)
        return PITCH_MIN_ANGLE;
    return pitch_target;
}

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
static CANInstance *rc_can_tx;        // 遥控原始值转发(CAN 0x352, 云台→底盘, 仅发送)
static RC_Trans_Data_s rc_trans_send; // 转发帧打包缓冲区
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
static uint8_t keyboard_shoot_allowed;
static uint8_t keyboard_shoot_last_key_count;
volatile uint8_t keyboard_shoot_fire_active;
#if YAW_STEP_TEST_ENABLE
static void YawStepTestSet(uint8_t keyboard_mode)
{
    uint8_t key_count = rc_data[TEMP].key_count[KEY_PRESS][Key_X];

    yaw_test_key_rise = 0.0f;
    if (keyboard_mode && key_count != yaw_test_last_key_count)
    {
        gimbal_cmd_send.yaw += YAW_STEP_TEST_ANGLE;
        yaw_test_key_rise = 1.0f;
    }

    yaw_test_last_key_count = key_count;
    yaw_test_target = gimbal_cmd_send.yaw;
}
#endif

void RobotCMDInit()
{
    // BMI088_Init_Config_s bmi088_config = {
    //     .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
    //     .work_mode = BMI088_BLOCK_TRIGGER_MODE,
    //     .spi_acc_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOA,
    //         .cs_pin = GPIO_PIN_4,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .acc_int_config = {
    //         .GPIOx = GPIOC,
    //         .GPIO_Pin = GPIO_PIN_4,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .spi_gyro_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOB,
    //         .cs_pin = GPIO_PIN_0,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .gyro_int_config = {
    //         .GPIO_Pin = GPIO_PIN_5,
    //         .GPIOx = GPIOC,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .heat_pwm_config = {
    //         .htim = &htim10,
    //         .channel = TIM_CHANNEL_1,
    //         .period = 1,
    //     },
    //     .heat_pid_config = {
    //         .Kp = 0.5,
    //         .Ki = 0,
    //         .Kd = 0,
    //         .DeadBand = 0.1,
    //         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //         .IntegralLimit = 100,
    //         .MaxOut = 100,
    //     },
    // };
    //bmi088_test = BMI088Register(&bmi088_config);
   rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    // vision_recv_data = VisionInit(&huart1); // 已停用(改动清单C): 旧VCP协议停用, USB由SP协议(sp_protocol)独占; vision_recv_data保持NULL仅供旧分支兼容

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);

    // 遥控原始值转发 CAN 实例: 云台板仅发送(0x352), 无人回发该 ID, rx 占位避免过滤器冲突
    CAN_Init_Config_s rc_tx_config = {
        .can_handle = &hcan1,
        .tx_id = 0x352,
        .rx_id = 0x352,
    };
    rc_can_tx = CANRegister(&rc_tx_config);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch =
        (PITCH_MAX_ANGLE + PITCH_MIN_ANGLE) * 0.5f;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],底盘跟随云台
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    }

    // 云台参数,确定云台控制数据
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[中],视觉模式
    {
        // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
        // ...
    }
    // 左侧开关状态为[下],纯遥控器拨杆控制(视觉模式左[中]已由VisionAimSet独占, 此处不再引用vision_recv_data避免空指针)
    if (switch_is_down(rc_data[TEMP].rc.switch_left))
    { // 按照摇杆的输出大小进行角度增量,增益系数需调整
        if (abs(rc_data[TEMP].rc.rocker_l_) > 20) // 左摇杆水平中位死区: 残留绝对值超过20才累加yaw,消除中位漂移
            gimbal_cmd_send.yaw += 0.001f * (float)rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
    }
    // 云台软件限位

    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    chassis_cmd_send.vx = 30.0f * (float)rc_data[TEMP].rc.rocker_r_; // _水平方向
    chassis_cmd_send.vy = 30.0f * (float)rc_data[TEMP].rc.rocker_r1; // 1数值方向

    // 发射参数
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],弹舱打开
        ;                                            // 弹舱舵机控制,待添加servo_motor模块,开启
    else
        ; // 弹舱舵机控制,待添加servo_motor模块,关闭

    // 摩擦轮控制,拨轮向上打为负,向下为正
    if (rc_data[TEMP].rc.dial < -100) // 向上超过100,打开摩擦轮
        shoot_cmd_send.friction_mode = FRICTION_ON;
    else
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    if (rc_data[TEMP].rc.dial < -500)
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    else
        shoot_cmd_send.load_mode = LOAD_STOP;
    // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    shoot_cmd_send.shoot_rate = 16;

    // VarScope 桥接: 摇杆/拨轮原始值(中位残留验证), rc_data 由 RemoteControlInit 赋值, 进入本函数前已判空
    dbg_rc_rocker_l_ = (float)rc_data[TEMP].rc.rocker_l_;
    dbg_rc_rocker_l1 = (float)rc_data[TEMP].rc.rocker_l1;
    dbg_rc_rocker_r_ = (float)rc_data[TEMP].rc.rocker_r_;
    dbg_rc_rocker_r1 = (float)rc_data[TEMP].rc.rocker_r1;
    dbg_rc_dial      = (float)rc_data[TEMP].rc.dial;
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    uint8_t key_count;
    float keyboard_speed;

    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?7
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],底盘跟随云台
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    }
    gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
    {
    case 0:
        shoot_cmd_send.bullet_speed = 15;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = 18;
        break;
    default:
        shoot_cmd_send.bullet_speed = 30;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4) // E键设置发射模式
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    case 1:
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        break;
    case 2:
        shoot_cmd_send.load_mode = LOAD_3_BULLET;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    default:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 40;
        break;
    case 1:
        chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:

        break;

    default:

        break;
    }

    key_count = rc_data[TEMP].key_count[KEY_PRESS][Key_G]; // 切换发弹许可模式，上电后默认关闭发弹，按G后解锁发弹。

    keyboard_speed = KEYBOARD_CHASSIS_BASE_SPEED *
                     (float)chassis_cmd_send.chassis_speed_buff * 0.01f;
    if (rc_data[TEMP].key[KEY_PRESS].shift)
        keyboard_speed *= KEYBOARD_SHIFT_SPEED_SCALE;

    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].d * keyboard_speed -
                          rc_data[TEMP].key[KEY_PRESS].a * keyboard_speed;
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].w * keyboard_speed -
                          rc_data[TEMP].key[KEY_PRESS].s * keyboard_speed;

    if (key_count != keyboard_shoot_last_key_count)
        keyboard_shoot_allowed = !keyboard_shoot_allowed;
    keyboard_shoot_last_key_count = key_count;

    keyboard_shoot_fire_active = 0;
    if (!keyboard_shoot_allowed)
    {
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    else if (rc_data[TEMP].mouse.press_l)
    {
        // The left mouse button reuses the friction-wheel start action and adds feeding.
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        shoot_cmd_send.shoot_rate = 8;
        keyboard_shoot_fire_active = 1;
    }
    else
    {
        // F controls pre-spin; feeding is only enabled by the left mouse button.
        shoot_cmd_send.shoot_mode = shoot_cmd_send.friction_mode == FRICTION_ON
                                         ? SHOOT_ON
                                         : SHOOT_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
}

/**
 * @brief 左开关[中]视觉自瞄(改动清单C-绝对角直导). 视觉端(tongjiceshi)经 SP 下发
 *        yaw/pitch 为【世界系绝对角度】(含目标角与机械偏差补偿, 视觉端已用云台四元数
 *        旋转融合当前姿态), 故直接作为云台/底盘目标下发, 禁止再叠加自身 INS 基准
 *        (绝对角 + INS 会双倍偏). 符号仍在消费口取反(见 VISION_YAW_SIGN).
 *        yaw 得到云台绝对目标后经 CAN1 0x350 发底盘板执行(底盘 yaw 环跟随该角度);
 *        pitch 云台板本地闭环(经 gimbal_cmd_send.pitch 送至云台角度环).
 *        gimbal_mode 仅由右开关决定(GYRO/FREE), 视觉 mode 仅由左开关[中]门控, 两路不可混淆.
 */
static void VisionAimSet(void)
{
    const SP_VisionToGimbal_t *rx_sp = SPProtocolGetRecv();

    // 掉帧/从未收到有效帧/视觉离线(Daemon 25ms 未喂狗)时云台保持上一周期目标, 掉帧不导致跳变;
    // 射击侧: 立即停止拨弹(load_mode=LOAD_STOP), 杜绝旧连发指令残留导致无差别出弹;
    // 摩擦轮保留当前状态(开火中掉线则继续预转, 不发射弹丸, 视觉重连后可立即出弹;
    // 若本就未开火则维持关闭), 兼顾安全与响应速度.
    if (rx_sp == NULL || !SPProtocolIsOnline())
    {
        // 非视觉分支: 清零速度/加速度前馈防残留注入
        gimbal_cmd_send.yaw_vel = 0.0f;
        gimbal_cmd_send.yaw_acc = 0.0f;
        shoot_cmd_send.load_mode = LOAD_STOP;
        return;
    }

    // mode==0(视觉不控制): 云台保持上一周期目标, 射击侧不动(保持当前状态)
    if (rx_sp->mode == 0)
    {
        // 非视觉分支: 清零速度/加速度前馈防残留注入
        gimbal_cmd_send.yaw_vel = 0.0f;
        gimbal_cmd_send.yaw_acc = 0.0f;
        return;
    }

    // mode==1(控制云台不开火) / mode==2(控制云台且开火): 云台均跟随视觉目标.
    // 开火完全由上位机(视觉端) mode 决定, 不再由遥控器拨轮触发:
    //   mode==1: 云台跟随 + 显式停射击(摩擦轮/拨弹全关);
    //   mode==2: 云台跟随 + 开火(摩擦轮预转 + 拨弹连发, 射频 8 发/s);
    //   拨轮 dial 不再参与本分支(开火权移交上位机), 急停仍由 EmergencyHandler 保留.
    // yaw方向符号只在消费口取反(陀螺仪yaw坐标与视觉坐标系相反)
    // pitch方向符号同样在消费口取反(实车验证: 装甲板在下但pitch上转, 命令方向反, 按 yaw 同法取反)
    static const float VISION_YAW_SIGN = -1.0f;
    static const float VISION_PITCH_SIGN = -1.0f;

    // yaw: 绝对目标(rad -> deg)直接下发, 由底盘板执行; 视觉已含当前姿态融合, 不再叠 INS
    gimbal_cmd_send.yaw = VISION_YAW_SIGN * rx_sp->yaw * RAD_2_DEGREE;
    dbg_vision_yaw_cmd = VISION_YAW_SIGN * rx_sp->yaw * RAD_2_DEGREE; // VarScope: 视觉SP下发yaw命令(deg)
    // yaw 速度前馈: 视觉下发(rad/s) -> deg/s(与底盘速度环反馈 yaw_gyro 同域), 经 0x350 透传底盘
    gimbal_cmd_send.yaw_vel = VISION_YAW_SIGN * rx_sp->yaw_vel * RAD_2_DEGREE;
    // yaw 加速度前馈: 视觉下发(rad/s^2) 原样透传, 经 0x350 送底盘电流前馈(K_ACC_YAW 待标定)
    gimbal_cmd_send.yaw_acc = VISION_YAW_SIGN * rx_sp->yaw_acc;
    // pitch: 绝对目标(rad -> deg)直接下发, 送云台角度环闭环; 不再叠 INS.Pitch
    //        软件限幅由云台 app(gyro_fdb PITCH)统一处理; 方向按实车验证取反
    gimbal_cmd_send.pitch = VISION_PITCH_SIGN * rx_sp->pitch * RAD_2_DEGREE;
    dbg_vision_pitch = rx_sp->pitch;                                                       // VarScope: 视觉SP下发pitch原始值(rad)
    dbg_vision_pitch_cmd = VISION_PITCH_SIGN * rx_sp->pitch * RAD_2_DEGREE;                // VarScope: 视觉SP下发pitch命令(deg)

    // 开火与否由上机位 mode 决定:
    if (rx_sp->mode == 2)
    {
        // 视觉端已判定可开火(tracking 且 plan.fire): 摩擦轮预转 + 拨弹连发
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        shoot_cmd_send.shoot_rate = 8;
        shoot_cmd_send.bullet_speed = SMALL_AMU_30; // 自瞄固定 18m/s 弹速档(摩擦轮 FRICTION_SPEED_REF_18 = 20000 deg/s)
    }
    else
    {
        // mode==1: 云台跟随但显式停射击, 杜绝旧指令残留连发
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        robot_state = ROBOT_READY;
        if (!switch_is_up(rc_data[TEMP].rc.switch_left))
            shoot_cmd_send.shoot_mode = SHOOT_ON;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

#ifdef GIMBAL_BOARD
/**
 * @brief 遥控原始值转发: 帧格式与 CANComm 协议完全一致(head+datalen+data+crc8(data)+tail),
 *        底盘板 CANComm(0x352) 可直接解包; 12 字节数据 + 4 字节帧头 = 16 字节, 分 2 包 8 字节发送
 */
static void RcTransSend(void)
{
    static uint8_t raw[sizeof(RC_Trans_Data_s) + CAN_COMM_OFFSET_BYTES];
    static uint8_t crc8;
    static uint8_t send_len;

    raw[0] = CAN_COMM_HEADER;
    raw[1] = sizeof(RC_Trans_Data_s);
    memcpy(raw + 2, &rc_trans_send, sizeof(RC_Trans_Data_s));
    crc8 = crc_8((uint8_t *)&rc_trans_send, sizeof(RC_Trans_Data_s));
    raw[2 + sizeof(RC_Trans_Data_s)] = crc8;
    raw[sizeof(RC_Trans_Data_s) + CAN_COMM_OFFSET_BYTES - 1] = CAN_COMM_TAIL;

    for (size_t i = 0; i < sizeof(RC_Trans_Data_s) + CAN_COMM_OFFSET_BYTES; i += 8)
    {
        send_len = sizeof(RC_Trans_Data_s) + CAN_COMM_OFFSET_BYTES - i >= 8 ? 8 : sizeof(RC_Trans_Data_s) + CAN_COMM_OFFSET_BYTES - i;
        CANSetDLC(rc_can_tx, send_len);
        memcpy(rc_can_tx->tx_buff, raw + i, send_len);
        CANTransmit(rc_can_tx, 1);
    }
}
#endif

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试/键鼠/视觉
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
        RemoteControlSet();
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
        MouseKeySet();
    else // 遥控器左侧开关状态为[中],视觉自瞄(触发源=左[中], 改动清单C)
        VisionAimSet();

#if YAW_STEP_TEST_ENABLE
    YawStepTestSet(switch_is_up(rc_data[TEMP].rc.switch_left));
#endif

    gimbal_cmd_send.pitch = LimitPitchTarget(gimbal_cmd_send.pitch);

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // VarScope: 实际写入云台的 yaw 目标(deg), 每周期末端锁存最终值
    dbg_gimbal_yaw_ref = -gimbal_cmd_send.yaw;

    // SP 协议上报模式仅由左开关[中]门控(视觉mode); 云台 gimbal_mode 由右开关决定, 两路不可混淆
    SPProtocolSetMode(switch_is_mid(rc_data[TEMP].rc.switch_left) ? 1 : 0);
    // 弹速上报: 裁判系统已反馈真实弹速(>0.5 m/s)时上报真实值, 否则维持占位 15
    if (dbg_bullet_speed > 0.5f)
        SPProtocolSetBullet(dbg_bullet_speed, 0);
    else
        SPProtocolSetBullet(15.0f, 0);

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);

    // 遥控原始值转发: 串口回调持续刷新 rc_data[TEMP].rc.*, 每周期打包经 0x352 发底盘板供 dbg_rc_* 观测
    rc_trans_send.rocker_l_ = rc_data[TEMP].rc.rocker_l_;
    rc_trans_send.rocker_l1 = rc_data[TEMP].rc.rocker_l1;
    rc_trans_send.rocker_r_ = rc_data[TEMP].rc.rocker_r_;
    rc_trans_send.rocker_r1 = rc_data[TEMP].rc.rocker_r1;
    rc_trans_send.dial      = rc_data[TEMP].rc.dial;
    rc_trans_send.switch_left  = rc_data[TEMP].rc.switch_left;
    rc_trans_send.switch_right = rc_data[TEMP].rc.switch_right;
    RcTransSend();
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
