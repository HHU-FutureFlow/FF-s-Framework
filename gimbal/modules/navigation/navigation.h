#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "navigation_protocol.h"

#include <stdint.h>

typedef struct
{
    /* 运行期可在 Live Watch 修改的导航通信参数。 */
    uint8_t tx_enabled;
    uint8_t test_mode;
    uint16_t speed_tx_period_ms;
    uint16_t turret_tx_period_ms;
    uint16_t rx_timeout_ms;
    float test_vx;
    float test_vy;
    float test_wz;
} NavigationConfig_s;

typedef struct
{
    uint32_t speed_tx_frame_count;
    uint32_t turret_tx_frame_count;
    uint32_t tx_busy_count;
    uint32_t rx_frame_count;
    uint32_t rx_value_error_count;
    uint32_t last_speed_tx_time_ms;
    uint32_t last_turret_tx_time_ms;
    uint32_t last_rx_time_ms;
    uint32_t yaw_target_rx_frame_count;
    uint32_t yaw_target_rx_value_error_count;
    uint32_t last_yaw_target_rx_time_ms;
    NavigationVelocity_s last_tx_velocity;
    NavigationVelocity_s last_rx_velocity;
    float last_tx_turret_yaw;
    float last_rx_yaw_target_rad;
    uint8_t last_speed_flags;
    uint8_t last_turret_flags;
} NavigationDebug_s;

extern volatile NavigationConfig_s navigation_config;
extern volatile NavigationDebug_s navigation_debug;

void NavigationInit(void);
void NavigationTask(void);
/* 写入底盘实际速度反馈；速度单位为 m/s，时间戳单位为 us。 */
void NavigationSetSpeedFeedback(float vx, float vy, float wz,
                                uint8_t flags, uint32_t sample_time_us);
/* 将 A6 三个速度有效位清零；下次发送的无效速度会归零。 */
void NavigationInvalidateSpeedFeedback(void);
/* 写入 A7 云台 yaw 反馈；yaw 单位为 rad，valid 为 0 时发送无效标志。 */
void NavigationSetTurretYaw(float yaw_rad, uint8_t valid, uint32_t sample_time_us);
/* IMU 姿态首次有效时锁存启动参考航向，单位为连续角度 deg。 */
void NavigationSetYawReference(float yaw_zero_deg);
/* 复制最新 A5 命令；仅在命令未超时时返回 1。 */
uint8_t NavigationGetCommand(NavigationVelocity_s *command);
uint8_t NavigationCommandIsOnline(void);
/* 复制最新 A9 目标；要求已建立 IMU 参考航向且至少收到一帧有效 A9。 */
uint8_t NavigationGetYawTarget(float *yaw_target_rad);
uint8_t NavigationGetYawReference(float *yaw_zero_deg);

#endif
