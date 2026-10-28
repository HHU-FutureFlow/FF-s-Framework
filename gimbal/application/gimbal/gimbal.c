#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "can_comm.h"
#include <math.h>

/* ======================= Pitch 重力补偿前馈 ==========================
 * 物理模型: 负载质心偏离转轴,重力矩随俯仰角呈余弦规律
 *      tau_gravity(theta) = m * g * L * cos(theta - phi)
 *    theta : IMU 解算的 pitch 角(度, 水平为 0, 抬头为正)
 *    phi   : 质心-转轴连线相对水平方向的相位偏置(度), 即配重/安装角偏差
 * 工程实现: 用一阶余弦模型拟合"角度 - 稳态控制量"数据, 直接前馈到串级末端
 *      u_ff(theta) = PITCH_GRAVITY_K * cos(theta - PITCH_COG_ANGLE_OFFSET)
 *   其中 K 为折算到 DJI 控制量的等效系数(单位: 控制量计数),
 *   由 MATLAB 脚本 fit_gravity_K.m 对 Ozone 采样数据做最小二乘拟合得到。
 *
 * 标定流程:
 *   1) 理论初值: theory_K.m 按 m(kg)、L(m) 与 GM6020 转矩系数估算 K, 作为起调点;
 *      双向匀速扫角法 或 稳态静差法, 用 Ozone 采样 (角度, 稳态控制量),
 *      导出 CSV 后由 fit_gravity_K.m 拟合出 K 与 offset, 写回下方两个宏。
 * 量级参考: 设 m ~ 1.0kg、L ~ 0.10m, 则 tau ~ 1.0*9.8*0.10 = 0.98 N.m;
 *   GM6020 额定转矩 1.2 N.m 对应控制量约 16384, 则 K 的理论量级约
 *   0.98/1.2*16384 ~ 1.3e4。实际受配重、摩擦与力矩系数标定影响会有较大出入,
 *   务必以实测拟合为准。保守做法: 先用理论值的 30% 左右小步试探, 确认
 *   补偿方向正确后再整定。
 *
 * 安全提示: 本前馈叠加在串级末端(pitch 未启用电流环时为最终下发量),
 *   受 GM6020 +-16384 限幅保护(dji_motor.c)。若 K 符号错误, 表现为
 *   重力方向抖动/低头加剧, 须立即将 K 置 0 并复核拟合符号与角度正方向。
 * ==================================================================== */
#define PITCH_GRAVITY_K 10000.0f         // 重力补偿等效系数(控制量), 北理纯前馈法标定 K=8000
#define PITCH_COG_ANGLE_OFFSET -100.0f   // 质心相位偏置 phi(度), 北理纯前馈法标定 offset=-93

/* ======================= Pitch 换目标 kick 补偿 ========================
 * 目标角从静止跃迁瞬间, 注入一次短时输出(200), 克服静摩擦/启动粘滞:
 *   - 仅检测"静止 -> 运动"边沿(边沿触发), 触发一次后不再注入,
 *     直到目标重新静止并确认足够久, 下次跃迁才再次触发;
 *   - "静止"须连续 PITCH_KICK_STILL_MS 拍 Δref < 阈值才算数,
 *     避免长跃迁过程中单拍速率波动被误判为静止而重复触发;
 *   - 触发后进入 PITCH_KICK_LOCK_MS 拍冷却, 双保险保证一次跃迁最多注入一次;
 *   - 注入方向跟随目标变化方向(sign(Δref)), 若实车换向时先反冲,
 *     说明符号与电机方向约定相反, 将 copysignf 改为 -copysignf 即可。
 * 实现: kick 量临时叠加进 pitch_gravity_ff, 经 current_feedforward_ptr
 *   注入串级末端(速度环输出之后), 下一拍重力前馈重算后 kick 自动消失,
 *   等效"只注入一拍"。受 GM6020 +-16384 限幅保护。
 * ==================================================================== */
#define PITCH_KICK_THRESHOLD 0.1f   // 运动判定阈值(deg/拍)
#define PITCH_KICK_STILL_MS  0     // 静止确认时长(拍)
#define PITCH_KICK_LOCK_MS   0    // 触发后冷却(拍)
#define PITCH_KICK_OUT       300.0f // 注入输出量(控制量)

/* ======================= Yaw 速度+加速度前馈 ==========================
 * 视觉端(SP)下发 yaw 角速度/角加速度, VisionAimSet 消费后写入
 * gimbal_cmd_send(Gimbal_Ctrl_Cmd_s), 经消息中心透传本任务, 组帧 0x350 发底盘:
 *   yaw_vel(deg/s)  -> 底盘 yaw 速度环前馈(SPEED_FEEDFORWARD);
 *   yaw_acc(rad/s^2) -> 底盘 yaw 电流前馈(CURRENT_FEEDFORWARD, 底盘侧乘 K_ACC_YAW).
 * 方向符号已在 VisionAimSet 消费口取反(VISION_YAW_SIGN), 此处仅做增益透传.
 * K_FF_YAW 初值 1.0(纯透传), 与底盘侧 K_ACC_YAW 一起待实车标定:
 *   小步试探(0.3 -> 0.6 -> 1.0), 确认跟随改善且无振荡后逐级加大;
 *   符号方向错误表现为跟随反向加剧/振荡, 应立即置 0 复核.
 * ==================================================================== */
#define K_FF_YAW 0.0f   // yaw 速度前馈增益(待实车标定)

#ifndef DEG2RAD
#define DEG2RAD(deg) ((deg) * DEGREE_2_RAD) // 度 -> 弧度(复用 general_def.h 的 DEGREE_2_RAD)
#endif

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

// pitch 重力补偿前馈量(控制量计数), 由 GimbalTask 每周期更新,
// 经 pitch_motor 的 current_feedforward_ptr 叠加到串级末端
static float pitch_gravity_ff = 0.0f;

// ---- 换目标 kick 补偿状态(仅 GimbalTask 使用) ----
static float last_pitch_ref = 0.0f;       // 上一拍 pitch 参考(限幅后)
static uint16_t pitch_kick_still_cnt = 0; // 连续静止拍数
static uint16_t pitch_kick_lock = 0;      // 触发后冷却拍数

// ------------------- VarScope 调试观测变量 -------------------
// VarScope 不支持观察指针与结构体成员, 故用独立全局变量桥接,
// 供重力补偿 K 标定采样(角度-稳态控制量)使用; 标定结束后可整段删除。
volatile float dbg_pitch_angle = 0.0f; // IMU 解算的 pitch 角(度, 抬头为正)
volatile float dbg_pitch_gyro = 0.0f;  // pitch 轴角速度(deg/s, 与速度环反馈同源)
volatile float dbg_pitch_out = 0.0f;   // pitch 串级末端输出(控制量计数, 限幅前)
volatile float dbg_yaw_angle = 0.0f;   // yaw 相对角(底盘回传 yaw_feedback_recv.yaw_angle, 度)
volatile float dbg_pitch_ref = 0.0f;   // pitch 参考角(限幅后 gimbal_cmd_recv.pitch, 度)

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
                .Kp = 20.0,
                .Ki = 3.2,
                .Kd = 0.8,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 80,
                .Ki = 18.0,
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
            // 重力补偿前馈注入点: 叠加在串级末端(pitch 未启用电流环, 叠加后即最终下发量)
            .current_feedforward_ptr = &pitch_gravity_ff,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            // 使能末端前馈通道; 必须与 current_feedforward_ptr 成对设置,
            // 只置标志位而不填指针会解引用 NULL 导致硬件错误
            .feedforward_flag = CURRENT_FEEDFORWARD,
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
    float yaw_cur_deg, yaw_diff_deg; // yaw_ref 最短弧归一化临时量(见下方注释)
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    gimbal_cmd_recv.pitch = LimitPitchAngle(gimbal_cmd_recv.pitch);
    // yaw_ref 最短弧归一化(防跨 ±180° 边界套圈):
    // 底盘 yaw 角度环反馈为 INS YawTotalAngle(累积角, 可多圈), 目标 yaw 来自视觉 SP
    // (单圈域 -180~180, VisionAimSet 绝对角直导)或遥控增量累积。若目标与当前反馈直接
    // 相减, 跨 ±180° 边界时 Err 取长弧(例: 当前 -178°, 目标 3°, Err=+181° 而非 -179°)。
    // 此处把目标换算为与当前累积反馈角最近的多圈等效角, 使底盘 PID 误差自然落在
    // [-180, 180] 最短弧。对遥控增量模式同样安全: 目标与当前差 < 180° 时换算结果不变。
    yaw_cur_deg = GYRO2GIMBAL_DIR_YAW * gimba_IMU_data->YawTotalAngle;
    yaw_diff_deg = gimbal_cmd_recv.yaw - yaw_cur_deg;
    yaw_diff_deg = fmodf(yaw_diff_deg + 180.0f, 360.0f);
    if (yaw_diff_deg < 0.0f)
        yaw_diff_deg += 360.0f;
    yaw_diff_deg -= 180.0f; // 归一化到 [-180, 180]
    yaw_cmd_send.yaw_ref = yaw_cur_deg + yaw_diff_deg;
    yaw_cmd_send.yaw_angle = GYRO2GIMBAL_DIR_YAW * gimba_IMU_data->YawTotalAngle;
    yaw_cmd_send.yaw_gyro = GYRO2GIMBAL_DIR_YAW * gimba_IMU_data->Gyro[2];
    yaw_cmd_send.mode = (uint8_t)gimbal_cmd_recv.gimbal_mode;
    yaw_cmd_send.enable =gimbal_cmd_recv.gimbal_mode != GIMBAL_ZERO_FORCE;
    // yaw 速度/加速度前馈透传: 视觉下发经 cmd 写入 gimbal_cmd_recv, 组帧转发底盘
    //   yaw_vel(deg/s, 与底盘速度环反馈同域)  乘 K_FF_YAW 增益;
    //   yaw_acc(rad/s^2) 原样透传, 由底盘侧 K_ACC_YAW 缩放注入电流前馈
    // 非视觉分支(VisionAimSet 已清零)时此处自然为 0, 不产生前馈注入
    yaw_cmd_send.yaw_vel = K_FF_YAW * gimbal_cmd_recv.yaw_vel;
    yaw_cmd_send.yaw_acc = gimbal_cmd_recv.yaw_acc;

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

    // pitch 重力补偿前馈力矩
    // 根据IMU姿态解算出当前配重下的重力矩, 用一阶余弦模型拟合(详见文件头注释):
    //   tau_gravity(theta) = m*g*L*cos(theta - phi)
    //   theta 取 IMU 解算的 Pitch(度), phi = PITCH_COG_ANGLE_OFFSET
    //   K、phi 由 MATLAB 标定脚本给出, 初值 0 时该前馈无输出(等效未启用)
    // 该值经 pitch_motor 的 current_feedforward_ptr 叠加在串级末端,
    // 因 pitch 未启用电流环, 叠加后即为最终下发量(受 GM6020 +-16384 限幅)
    // 重力补偿按反馈角(IMU实测Pitch)计算: 重力矩只取决于真实姿态, 稳态/动态均与
    // 实际重力一致; 曾试目标角方案(切目标瞬间前馈提前到位)但到达目标附近出现过冲回摆,
    // 实车验证后回退反馈角方案
    pitch_gravity_ff = PITCH_GRAVITY_K * cosf(DEG2RAD(gimba_IMU_data->Pitch - PITCH_COG_ANGLE_OFFSET));

    // ---- 换目标 kick 补偿: 静止->跃迁边沿触发, 单次注入 ----
    // Δref 基于限幅后的 pitch 参考(gimbal_cmd_recv.pitch 已在本任务开头限幅)
    {
        float delta_ref = gimbal_cmd_recv.pitch - last_pitch_ref;
        last_pitch_ref  = gimbal_cmd_recv.pitch;

        uint8_t cur_moving = (fabsf(delta_ref) > PITCH_KICK_THRESHOLD) ? 1 : 0;

        if (cur_moving)
            pitch_kick_still_cnt = 0;                // 一有运动即清零
        else if (pitch_kick_still_cnt < 0xFFFF)
            pitch_kick_still_cnt++;                  // 连续静止累计

        // 触发: 正在运动 + 已连续静止确认 + 不在冷却期 -> 注入一次并锁定
        if (cur_moving && (pitch_kick_still_cnt >= PITCH_KICK_STILL_MS) && (pitch_kick_lock == 0))
        {
            pitch_gravity_ff -= copysignf(PITCH_KICK_OUT, delta_ref); // 方向跟随目标运动方向(实车验证: 原+号方向反, 表现为先偏移再回归, 已取反)
            pitch_kick_lock = PITCH_KICK_LOCK_MS;                     // 锁定, 长跃迁内不再触发
        }
        else if (pitch_kick_lock > 0)
        {
            pitch_kick_lock--;
        }
    }

    // VarScope 标定观测变量赋值
    //   dbg_pitch_out 取 speed_PID.Output: pitch 未启用电流环, 该值叠加前馈后
    //   即为最终下发量(K=0 空跑标定时, 即为纯 PID 输出, 可用于静差法拟合)
    dbg_pitch_angle = gimba_IMU_data->Pitch;
    dbg_pitch_gyro = gimba_IMU_data->Gyro[0];
    dbg_pitch_out = pitch_motor->motor_controller.speed_PID.Output;
    dbg_pitch_ref = gimbal_cmd_recv.pitch; // 限幅后的 pitch 参考角


    if (CANCommIsOnline(yaw_can_comm))
    {
        Yaw_Upload_Data_s *yaw_feedback = (Yaw_Upload_Data_s *)CANCommGet(yaw_can_comm);

        yaw_feedback_recv = *yaw_feedback;
        yaw_motor_single_round_angle = yaw_feedback_recv.yaw_motor_single_round_angle;
        dbg_yaw_angle = yaw_feedback_recv.yaw_angle; // VarScope 观测 yaw 相对角(仅在线时刷新)
    }


    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor_single_round_angle;
    gimbal_feedback_data.pitch_motor_single_round_angle = pitch_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
    
}
