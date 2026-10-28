/**
 ******************************************************************************
 * @file    sp_protocol.h
 * @author  HNU YueLu EC
 * @brief   同济视觉 SP 协议串口模块(经 MicroUSB 虚拟串口 VCP 与视觉端通信)
 *          按《FF步兵SP自瞄移植改动清单》从 HDRM 移植到 FF-s-Framework 云台板
 ******************************************************************************
 * @attention
 *  - 帧头 'S','P' + CRC16(查表法, init=0xffff, 与视觉端 tools/crc.cpp 完全一致)
 *  - 本模块使用 bsp_usb(VCP), 发送 GimbalToVision / 接收解析 VisionToGimbal
 *  - 同一时刻 USB VCP 的 rx_cbk 只能被一个模块占用, 启用本模块后需停用
 *    master_process 的 VISION_USE_VCP 分支(见 robot_def.h)
 ******************************************************************************
 */
#ifndef SP_PROTOCOL_H
#define SP_PROTOCOL_H

#include <stdint.h>

/* 下位机 -> 视觉: 云台姿态/弹道状态上报 (视觉端 io::GimbalToVision) */
#pragma pack(1)
typedef struct
{
    uint8_t head[2];    // 'S','P'
    uint8_t mode;       // 0:空闲 1:自瞄 2:小符 3:大符
    float q[4];         // 四元数, wxyz 顺序
    float yaw;          // 云台 yaw 角
    float yaw_vel;      // yaw 角速度
    float pitch;        // 云台 pitch 角
    float pitch_vel;    // pitch 角速度
    float bullet_speed; // 弹速 m/s
    uint16_t bullet_count; // 累计发弹计数
    uint16_t crc16;     // 帧校验(小端, 计算范围=除本字段外全部)
} SP_GimbalToVision_t;

/* 视觉 -> 下位机: 云台控制指令 (视觉端 io::VisionToGimbal) */
typedef struct
{
    uint8_t head[2];    // 'S','P'
    uint8_t mode;       // 0:不控制 1:控制云台不开火 2:控制云台且开火
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;     // 帧校验(小端)
} SP_VisionToGimbal_t;
#pragma pack()

/**
 * @brief 初始化 SP 协议模块(注册 USB VCP 接收回调)
 * @note  在 StartINSTASK 中 INS_Init 之后调用; 与 master_process 的 VCP 分支互斥
 */
void SPProtocolInit(void);

/**
 * @brief 发送一帧云台姿态上报(应在 1kHz 周期任务中调用, 如 INS 任务)
 * @note  姿态数据默认取自 INS(四元数 wxyz / Yaw/Pitch / Gyro),
 *        单位: 角度=rad, 角速度=rad/s; 反馈源全部来自云台 C 板
 */
void SPProtocolSend(void);

/**
 * @brief 设置上报模式(0:空闲 1:自瞄 2:小符 3:大符)
 */
void SPProtocolSetMode(uint8_t mode);

/**
 * @brief 设置弹速与累计发弹数
 */
void SPProtocolSetBullet(float speed, uint16_t count);

/**
 * @brief 获取最近一帧视觉控制指令(校验通过才更新)
 */
const SP_VisionToGimbal_t *SPProtocolGetRecv(void);

/**
 * @brief 视觉在线状态查询(在线返回非0)
 * @note  内部基于 sp_daemon 判断: 距上次有效帧 25ms 内视为在线;
 *        离线/未初始化时返回 0, 供消费口做视觉掉线保护(停止累加旧增量)
 */
uint8_t SPProtocolIsOnline(void);

#endif // !SP_PROTOCOL_H
