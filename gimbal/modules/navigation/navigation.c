#include "navigation.h"

#include "main.h"
#include "pc_comm.h"

#include <stddef.h>
#include <string.h>

volatile NavigationConfig_s navigation_config = {
    .tx_enabled = 1u,
    .test_mode = 0u,
    .speed_tx_period_ms = 20u,
    .turret_tx_period_ms = 10u,
    .rx_timeout_ms = 200u,
    .test_vx = 0.125f,
    .test_vy = -0.250f,
    .test_wz = 0.500f,
};

volatile NavigationDebug_s navigation_debug;

typedef struct
{
    NavigationVelocity_s velocity;
    uint32_t sample_time_us;
    uint8_t flags;
} NavigationSpeedFeedback_s;

typedef struct
{
    float yaw;
    uint32_t sample_time_us;
    uint8_t valid;
} NavigationTurretFeedback_s;

static volatile NavigationVelocity_s navigation_command;
static volatile NavigationYawTarget_s navigation_yaw_target;
static NavigationSpeedFeedback_s navigation_speed_feedback;
static NavigationTurretFeedback_s navigation_turret_feedback;
static uint8_t navigation_yaw_reference_ready;
static float navigation_yaw_zero_deg;

static void NavigationCommandHandler(const uint8_t *frame, uint16_t frame_length)
{
    NavigationVelocity_s velocity;

    if (NavigationProtocolDecodeCommand(frame, frame_length, &velocity) == 0u)
    {
        navigation_debug.rx_value_error_count++;
        return;
    }

    /* USB 回调上下文只更新缓存；最终控制权由 RobotCMDTask 仲裁。 */
    navigation_command.vx = velocity.vx;
    navigation_command.vy = velocity.vy;
    navigation_command.wz = velocity.wz;
    navigation_debug.last_rx_velocity = velocity;
    navigation_debug.last_rx_time_ms = HAL_GetTick();
    navigation_debug.rx_frame_count++;
}

static void NavigationYawTargetHandler(const uint8_t *frame, uint16_t frame_length)
{
    NavigationYawTarget_s target;

    if (NavigationProtocolDecodeYawTarget(frame, frame_length, &target) == 0u)
    {
        navigation_debug.yaw_target_rx_value_error_count++;
        return;
    }

    /* 尚未获得可靠 IMU 起始航向时不缓存目标，避免初始化完成后突发执行旧指令。 */
    if (navigation_yaw_reference_ready == 0u)
    {
        return;
    }

    navigation_yaw_target = target;
    navigation_debug.last_rx_yaw_target_rad = target.yaw_target_rad;
    navigation_debug.last_yaw_target_rx_time_ms = HAL_GetTick();
    navigation_debug.yaw_target_rx_frame_count++;
}

static void NavigationRawFrameHandler(const uint8_t *frame, uint16_t frame_length)
{
    if ((frame == NULL) || (frame_length == 0u))
    {
        return;
    }

    /* PCComm 已按帧头和长度交付，此处按帧头分给 A5/A9 专用解码器。 */
    if (frame[0] == NAVIGATION_COMMAND_HEADER)
    {
        NavigationCommandHandler(frame, frame_length);
    }
    else if (frame[0] == NAVIGATION_YAW_TARGET_HEADER)
    {
        NavigationYawTargetHandler(frame, frame_length);
    }
}

void NavigationInit(void)
{
    memset((void *)&navigation_debug, 0, sizeof(navigation_debug));
    memset(&navigation_speed_feedback, 0, sizeof(navigation_speed_feedback));
    memset(&navigation_turret_feedback, 0, sizeof(navigation_turret_feedback));
    memset((void *)&navigation_command, 0, sizeof(navigation_command));
    memset((void *)&navigation_yaw_target, 0, sizeof(navigation_yaw_target));
    navigation_yaw_reference_ready = 0u;
    navigation_yaw_zero_deg = 0.0f;
    PCCommRegisterNavigationHandler(NavigationRawFrameHandler);
}

static uint8_t NavigationTrySendSpeed(uint32_t now)
{
    NavigationSpeedFeedback_s feedback;
    uint8_t frame[NAVIGATION_SPEED_FEEDBACK_FRAME_SIZE];
    uint8_t flags;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    feedback = navigation_speed_feedback;
    if (primask == 0u)
    {
        __enable_irq();
    }

    if (navigation_config.test_mode != 0u)
    {
        feedback.velocity.vx = navigation_config.test_vx;
        feedback.velocity.vy = navigation_config.test_vy;
        feedback.velocity.wz = navigation_config.test_wz;
        feedback.sample_time_us = now * 1000u;
        feedback.flags = NAVIGATION_FLAG_LINEAR_VALID | NAVIGATION_FLAG_ANGULAR_VALID;
    }

    flags = (uint8_t)(feedback.flags &
                      (NAVIGATION_FLAG_LINEAR_VALID | NAVIGATION_FLAG_ANGULAR_VALID));
    if (now < 1000u)
    {
        flags |= NAVIGATION_FLAG_TIME_RESET;
    }
    if ((flags & NAVIGATION_FLAG_LINEAR_VALID) == 0u)
    {
        feedback.velocity.vx = 0.0f;
        feedback.velocity.vy = 0.0f;
    }
    if ((flags & NAVIGATION_FLAG_ANGULAR_VALID) == 0u)
    {
        feedback.velocity.wz = 0.0f;
    }

    NavigationProtocolPackSpeedFeedback(feedback.sample_time_us, flags,
                                        &feedback.velocity, frame);
    if (PCCommSendBytes(frame, sizeof(frame)) != PC_COMM_SEND_OK)
    {
        return 0u;
    }

    navigation_debug.last_tx_velocity = feedback.velocity;
    navigation_debug.last_speed_flags = flags;
    navigation_debug.last_speed_tx_time_ms = now;
    navigation_debug.speed_tx_frame_count++;
    return 1u;
}

static uint8_t NavigationTrySendTurret(uint32_t now)
{
    NavigationTurretFeedback_s feedback;
    uint8_t frame[NAVIGATION_TURRET_FEEDBACK_FRAME_SIZE];
    uint8_t flags = 0u;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    feedback = navigation_turret_feedback;
    if (primask == 0u)
    {
        __enable_irq();
    }

    if (feedback.valid != 0u)
    {
        flags |= NAVIGATION_FLAG_LINEAR_VALID;
    }
    else
    {
        feedback.yaw = 0.0f;
    }
    if (now < 1000u)
    {
        flags |= NAVIGATION_FLAG_TIME_RESET;
    }

    NavigationProtocolPackTurretFeedback(feedback.sample_time_us, flags,
                                         feedback.yaw, frame);
    if (PCCommSendBytes(frame, sizeof(frame)) != PC_COMM_SEND_OK)
    {
        return 0u;
    }

    navigation_debug.last_tx_turret_yaw = feedback.yaw;
    navigation_debug.last_turret_flags = flags;
    navigation_debug.last_turret_tx_time_ms = now;
    navigation_debug.turret_tx_frame_count++;
    return 1u;
}

void NavigationTask(void)
{
    static uint8_t prefer_turret_when_both_due;
    uint32_t now;
    uint16_t speed_period;
    uint16_t turret_period;
    uint8_t speed_due;
    uint8_t turret_due;

    if (navigation_config.tx_enabled == 0u)
    {
        return;
    }

    now = HAL_GetTick();
    speed_period = navigation_config.speed_tx_period_ms;
    turret_period = navigation_config.turret_tx_period_ms;
    if (speed_period == 0u)
    {
        speed_period = 1u;
    }
    if (turret_period == 0u)
    {
        turret_period = 1u;
    }

    speed_due = (uint8_t)((uint32_t)(now - navigation_debug.last_speed_tx_time_ms) >= speed_period);
    turret_due = (uint8_t)((uint32_t)(now - navigation_debug.last_turret_tx_time_ms) >= turret_period);

    /* 两路同时到期时交替优先发送，避免任务周期恰为 20 ms 等情况导致一路长期饥饿。 */
    if ((speed_due != 0u) && (turret_due != 0u))
    {
        if (prefer_turret_when_both_due != 0u)
        {
            if (NavigationTrySendTurret(now) == 0u)
            {
                navigation_debug.tx_busy_count++;
            }
            else
            {
                prefer_turret_when_both_due = 0u;
            }
        }
        else
        {
            if (NavigationTrySendSpeed(now) == 0u)
            {
                navigation_debug.tx_busy_count++;
            }
            else
            {
                prefer_turret_when_both_due = 1u;
            }
        }
    }
    else if (speed_due != 0u)
    {
        if (NavigationTrySendSpeed(now) == 0u)
        {
            navigation_debug.tx_busy_count++;
        }
    }
    else if ((turret_due != 0u) && (NavigationTrySendTurret(now) == 0u))
    {
        navigation_debug.tx_busy_count++;
    }
}

void NavigationSetSpeedFeedback(float vx, float vy, float wz,
                                uint8_t flags, uint32_t sample_time_us)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    navigation_speed_feedback.velocity.vx = vx;
    navigation_speed_feedback.velocity.vy = vy;
    navigation_speed_feedback.velocity.wz = wz;
    navigation_speed_feedback.flags = flags;
    navigation_speed_feedback.sample_time_us = sample_time_us;
    if (primask == 0u)
    {
        __enable_irq();
    }
}

void NavigationInvalidateSpeedFeedback(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    navigation_speed_feedback.flags = 0u;
    navigation_speed_feedback.velocity.vx = 0.0f;
    navigation_speed_feedback.velocity.vy = 0.0f;
    navigation_speed_feedback.velocity.wz = 0.0f;
    if (primask == 0u)
    {
        __enable_irq();
    }
}

void NavigationSetTurretYaw(float yaw_rad, uint8_t valid, uint32_t sample_time_us)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    navigation_turret_feedback.yaw = yaw_rad;
    navigation_turret_feedback.valid = valid;
    navigation_turret_feedback.sample_time_us = sample_time_us;
    if (primask == 0u)
    {
        __enable_irq();
    }
}

void NavigationSetYawReference(float yaw_zero_deg)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    /* 参考角只锁存一次，A9 始终解释为相对本次上电初始航向的绝对目标。 */
    if (navigation_yaw_reference_ready == 0u)
    {
        navigation_yaw_zero_deg = yaw_zero_deg;
        navigation_yaw_reference_ready = 1u;
    }
    if (primask == 0u)
    {
        __enable_irq();
    }
}

uint8_t NavigationGetCommand(NavigationVelocity_s *command)
{
    uint32_t primask;

    if (command == NULL)
    {
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    command->vx = navigation_command.vx;
    command->vy = navigation_command.vy;
    command->wz = navigation_command.wz;
    if (primask == 0u)
    {
        __enable_irq();
    }
    return NavigationCommandIsOnline();
}

uint8_t NavigationCommandIsOnline(void)
{
    uint32_t timeout = navigation_config.rx_timeout_ms;

    if (navigation_debug.rx_frame_count == 0u)
    {
        return 0u;
    }
    return (uint8_t)((uint32_t)(HAL_GetTick() - navigation_debug.last_rx_time_ms) <= timeout);
}

uint8_t NavigationGetYawTarget(float *yaw_target_rad)
{
    uint32_t primask;

    if (yaw_target_rad == NULL)
    {
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *yaw_target_rad = navigation_yaw_target.yaw_target_rad;
    if (primask == 0u)
    {
        __enable_irq();
    }

    return (uint8_t)((navigation_yaw_reference_ready != 0u) &&
                     (navigation_debug.yaw_target_rx_frame_count != 0u));
}

uint8_t NavigationGetYawReference(float *yaw_zero_deg)
{
    uint32_t primask;

    if (yaw_zero_deg == NULL)
    {
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *yaw_zero_deg = navigation_yaw_zero_deg;
    if (primask == 0u)
    {
        __enable_irq();
    }
    return navigation_yaw_reference_ready;
}
