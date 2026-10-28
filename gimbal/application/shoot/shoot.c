#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "can_comm.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r; // 摩擦轮电机
// static servo_instance *lid; 需要增加弹舱盖
static CANCommInstance *shoot_can_comm;

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息
float friction_l_speed;

// ------------------- VarScope 调试观测变量 -------------------
// 摩擦轮反馈(speed/cur)已在 dji_motor.c 内做一阶低通滤波, 此处直接桥接滤波后值;
// out 取电流环输出(最终下发电流量), loader 系列取底盘经 CAN 回传的实测值。
volatile float dbg_fric_l_speed = 0.0f;    // 左摩擦轮实测转速(滤波后, deg/s)
volatile float dbg_fric_r_speed = 0.0f;    // 右摩擦轮实测转速(滤波后, deg/s)
volatile float dbg_fric_l_ref = 0.0f;      // 左摩擦轮目标转速(speed_PID.Ref, deg/s)
volatile float dbg_fric_r_ref = 0.0f;      // 右摩擦轮目标转速(speed_PID.Ref, deg/s)
volatile float dbg_fric_l_cur = 0.0f;      // 左摩擦轮实测电流(滤波后, A 计数)
volatile float dbg_fric_r_cur = 0.0f;      // 右摩擦轮实测电流(滤波后, A 计数)
volatile float dbg_fric_l_out = 0.0f;      // 左摩擦轮电流环输出(给定电流, 限幅前)
volatile float dbg_fric_r_out = 0.0f;      // 右摩擦轮电流环输出(给定电流, 限幅前)
volatile float dbg_loader_speed = 0.0f;    // 拨盘实测转速(底盘回传, deg/s)
volatile float dbg_loader_cur = 0.0f;      // 拨盘实测电流(底盘回传)
volatile float dbg_loader_cur_ref = 0.0f;  // 拨盘给定电流 ref(底盘回传)

void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 8, // 20
                .Ki = 0.5, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 2;
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 3;  // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

        CANComm_Init_Config_s shoot_comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x314,
            .rx_id = 0x313,
        },
        .recv_data_len = sizeof(Shoot_Upload_Data_s),
        .send_data_len = sizeof(Shoot_Ctrl_Cmd_s),
        .daemon_count = 10,
    };
    shoot_can_comm = CANCommInit(&shoot_comm_conf);


    

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    //对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
   


    CANCommSend(shoot_can_comm, (uint8_t *)&shoot_cmd_recv);

   


    // // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetRef(friction_l, FRICTION_SPEED_REF_15);
            DJIMotorSetRef(friction_r, FRICTION_SPEED_REF_15);
            break;
        case SMALL_AMU_18:
            DJIMotorSetRef(friction_l, FRICTION_SPEED_REF_18);
            DJIMotorSetRef(friction_r, FRICTION_SPEED_REF_18);
            break;
        case SMALL_AMU_30:
            DJIMotorSetRef(friction_l, FRICTION_SPEED_REF_30);
            DJIMotorSetRef(friction_r, FRICTION_SPEED_REF_30);
            break;
        default: // 注释修正: 未接入裁判系统弹速时, 摩擦轮按 30m/s 档高速预转(40000 deg/s, 与 FRICTION_SPEED_REF_30 一致; 原注释'4000'有误, 不改转速逻辑)
            DJIMotorSetRef(friction_l, 40000);
            DJIMotorSetRef(friction_r, 40000);
            break;
        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    if (CANCommIsOnline(shoot_can_comm))
   {
    shoot_feedback_data = *(Shoot_Upload_Data_s *)CANCommGet(shoot_can_comm);
   }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
    friction_l_speed = friction_l->measure.speed_aps;

    // VarScope 观测变量赋值(摩擦轮反馈已在 dji_motor.c 低通滤波, loader 为底盘回传值)
    dbg_fric_l_speed = friction_l->measure.speed_aps;
    dbg_fric_r_speed = -friction_r->measure.speed_aps;
    dbg_fric_l_ref = friction_l->motor_controller.speed_PID.Ref;
    dbg_fric_r_ref = friction_r->motor_controller.speed_PID.Ref;
    dbg_fric_l_cur = (float)friction_l->measure.real_current;
    dbg_fric_r_cur = (float)friction_r->measure.real_current;
    dbg_fric_l_out = friction_l->motor_controller.current_PID.Output;
    dbg_fric_r_out = friction_r->motor_controller.current_PID.Output;
    dbg_loader_speed = shoot_feedback_data.loader_speed;
    dbg_loader_cur = shoot_feedback_data.loader_current;
    dbg_loader_cur_ref = shoot_feedback_data.loader_current_ref;
}
