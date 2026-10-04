#ifndef NAVIGATION_PROTOCOL_H
#define NAVIGATION_PROTOCOL_H

#include <stdint.h>

/* USB CDC 上的自定义导航固定帧；完整字段说明见 navigation.md。 */
#define NAVIGATION_COMMAND_HEADER 0xA5u
#define NAVIGATION_COMMAND_FRAME_SIZE 15u
#define NAVIGATION_YAW_TARGET_HEADER 0xA9u
#define NAVIGATION_YAW_TARGET_FRAME_SIZE 7u
#define NAVIGATION_SPEED_FEEDBACK_HEADER 0xA6u
#define NAVIGATION_SPEED_FEEDBACK_FRAME_SIZE 20u
#define NAVIGATION_TURRET_FEEDBACK_HEADER 0xA7u
#define NAVIGATION_TURRET_FEEDBACK_FRAME_SIZE 12u

#define NAVIGATION_FLAG_LINEAR_VALID 0x01u
#define NAVIGATION_FLAG_ANGULAR_VALID 0x02u
#define NAVIGATION_FLAG_TIME_RESET 0x80u

typedef struct
{
    /* A5 中为云台坐标系前/左速度；A6 中为底盘坐标系速度，单位均为 m/s。 */
    float vx;
    float vy;
    /* 绕共享 Z 轴的偏航角速度，单位 rad/s。 */
    float wz;
} NavigationVelocity_s;

typedef struct
{
    /* 相对 IMU 启动参考航向的云台绝对 yaw 目标，单位 rad。 */
    float yaw_target_rad;
} NavigationYawTarget_s;

uint16_t NavigationProtocolChecksum(const uint8_t *data, uint16_t length);
uint8_t NavigationProtocolDecodeCommand(const uint8_t *frame,
                                        uint16_t frame_length,
                                        NavigationVelocity_s *velocity);
uint8_t NavigationProtocolDecodeYawTarget(const uint8_t *frame,
                                          uint16_t frame_length,
                                          NavigationYawTarget_s *target);
void NavigationProtocolPackSpeedFeedback(uint32_t sample_time_us,
                                         uint8_t flags,
                                         const NavigationVelocity_s *velocity,
                                         uint8_t frame[NAVIGATION_SPEED_FEEDBACK_FRAME_SIZE]);
void NavigationProtocolPackTurretFeedback(uint32_t sample_time_us,
                                          uint8_t flags,
                                          float turret_yaw,
                                          uint8_t frame[NAVIGATION_TURRET_FEEDBACK_FRAME_SIZE]);

#endif
