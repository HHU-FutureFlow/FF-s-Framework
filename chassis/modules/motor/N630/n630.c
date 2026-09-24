#include "n630.h"
#include "power_control.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "general_def.h"

#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

enum
{
    N630_CAN_PACKET_SET_CURRENT = 1,
    N630_CAN_PACKET_STATUS = 9,
};

#define N630_CURRENT_REF_SCALE 819.2f   // same convention as C620/M3508: 16384 == 20A
#define N630_CURRENT_REF_LIMIT 16384.0f // keep the same practical current range as C620
#define N630_TORQUE_COEF 0.0003662109375f
#define N630_POWER_COEF 187.0f / 3591.0f / 9.55f
#define N630_POWER_K1 1.23e-07f
#define N630_POWER_K2 1.453e-07f
#define N630_POWER_CONSTANT 4.081f
/* N630 默认安装方向 */
#define N630_DEFAULT_MOTOR_REVERSE MOTOR_DIRECTION_REVERSE

static N630MotorInstance *s_motor_pool[N630_MOTOR_CNT] = {0};
static uint8_t s_motor_count = 0;

static float n630_read_ptr(const float *ptr, float fallback)
{
    return ptr ? *ptr : fallback;
}

static float n630_clampf(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

static int32_t n630_limit_i32(float value)
{
    value = n630_clampf(value, (float)INT32_MIN, (float)INT32_MAX);
    if (value >= 0.0f)
        return (int32_t)(value + 0.5f);
    return (int32_t)(value - 0.5f);
}

static int32_t n630_floor_i32(float value)
{
    int32_t iv = (int32_t)value;
    if ((float)iv > value)
        iv--;
    return iv;
}

static int32_t n630_get_i32_be(const uint8_t *src)
{
    return (int32_t)(((uint32_t)src[0] << 24) |
                     ((uint32_t)src[1] << 16) |
                     ((uint32_t)src[2] << 8) |
                     ((uint32_t)src[3]));
}

static int16_t n630_get_i16_be(const uint8_t *src)
{
    return (int16_t)(((uint16_t)src[0] << 8) | (uint16_t)src[1]);
}

static void n630_write_i32_be(uint8_t dst[8], int32_t value)
{
    uint32_t u = (uint32_t)value;
    dst[0] = (uint8_t)(u >> 24);
    dst[1] = (uint8_t)(u >> 16);
    dst[2] = (uint8_t)(u >> 8);
    dst[3] = (uint8_t)(u);
}

static uint32_t n630_pack_can_id(uint8_t controller_id, uint8_t packet_id)
{
    return ((uint32_t)packet_id << 8) | controller_id;
}

static void n630_register_instance(N630MotorInstance *motor)
{
    if (s_motor_count < N630_MOTOR_CNT)
        s_motor_pool[s_motor_count++] = motor;
}

static void n630_update_measure(N630MotorInstance *motor, int32_t rpm_raw, int16_t current_raw, uint8_t integrate_angle)
{
    N630_Motor_Measure_s *measure = &motor->measure;

    measure->last_ecd = measure->ecd;
    measure->speed_aps = (1.0f - N630_SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * N630_SPEED_SMOOTH_COEF * (float)rpm_raw;
    measure->real_current = (1.0f - N630_CURRENT_SMOOTH_COEF) * measure->real_current +
                            N630_CURRENT_SMOOTH_COEF * ((float)current_raw / 10.0f * N630_CURRENT_REF_SCALE);

    if (integrate_angle)
    {
        measure->total_angle += measure->speed_aps * motor->dt;
        measure->total_round = n630_floor_i32(measure->total_angle / 360.0f);
        measure->angle_single_round = measure->total_angle - (float)measure->total_round * 360.0f;
        if (measure->angle_single_round < 0.0f)
            measure->angle_single_round += 360.0f;
    }

    measure->ecd = (uint16_t)n630_limit_i32(measure->angle_single_round / 360.0f * 8192.0f);
    if (measure->ecd >= 8192U)
        measure->ecd = 8191U;
    measure->temperature = 0;
    measure->fault_code = 0;
}

static void N630MotorDecode(CANInstance *_instance)
{
    N630MotorInstance *motor;
    uint8_t *rxbuff;

    if (_instance == NULL || _instance->id == NULL || _instance->rx_len < 8)
        return;

    motor = (N630MotorInstance *)_instance->id;
    rxbuff = _instance->rx_buff;

    if (motor->daemon)
        DaemonReload(motor->daemon);

    if (!motor->measure.online)
    {
        motor->measure.online = 1;
        motor->feed_cnt = DWT->CYCCNT;
        motor->dt = 0.0f;
        n630_update_measure(motor, n630_get_i32_be(rxbuff), n630_get_i16_be(rxbuff + 4), 0);
        return;
    }

    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);
    n630_update_measure(motor, n630_get_i32_be(rxbuff), n630_get_i16_be(rxbuff + 4), 1);
}

static void N630MotorLostCallback(void *motor_ptr)
{
    N630MotorInstance *motor = (N630MotorInstance *)motor_ptr;

    if (motor == NULL || motor->motor_can_instance == NULL)
        return;

    motor->measure.online = 0;
    LOGWARNING("[n630] Motor lost, can bus [%d], id [%d]",
               motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2,
               motor->motor_can_instance->tx_id);
}

static uint8_t n630_send_current(N630MotorInstance *motor, float current_set)
{
    float current_a;

    if (motor == NULL || motor->motor_can_instance == NULL)
        return 0;

    current_set = n630_clampf(current_set, -N630_CURRENT_REF_LIMIT, N630_CURRENT_REF_LIMIT);
    current_a = current_set / N630_CURRENT_REF_SCALE;
    n630_write_i32_be(motor->motor_can_instance->tx_buff, n630_limit_i32(current_a * 1000.0f));
    return CANTransmit(motor->motor_can_instance, 1.0f);
}

static void n630_apply_power_limit(N630MotorInstance **motors, float *current_out, uint8_t count)
{
    float power_limit = GetPowerLimit();
    float initial_total_power = 0.0f;
    float initial_give_power[N630_MOTOR_CNT] = {0};
    float speed_rpm[N630_MOTOR_CNT] = {0};

    if (power_limit <= 0.0f || count == 0)
        return;

    for (uint8_t i = 0; i < count; ++i)
    {
        speed_rpm[i] = motors[i]->measure.speed_aps / RPM_2_ANGLE_PER_SEC;
        initial_give_power[i] = N630_POWER_K1 * current_out[i] * current_out[i] +
                                N630_POWER_K2 * speed_rpm[i] * speed_rpm[i] +
                                N630_POWER_COEF * speed_rpm[i] * current_out[i] * N630_TORQUE_COEF +
                                N630_POWER_CONSTANT;
        if (initial_give_power[i] > 0.0f)
            initial_total_power += initial_give_power[i];
    }

    if (initial_total_power <= power_limit)
        return;

    {
        float ratio = power_limit / initial_total_power;
        for (uint8_t i = 0; i < count; ++i)
        {
            float a, b, c, disc, target_power;

            if (initial_give_power[i] < 0.0f)
                continue;

            target_power = initial_give_power[i] * ratio;
            a = N630_POWER_K1;
            b = N630_TORQUE_COEF * N630_POWER_COEF * speed_rpm[i];
            c = N630_POWER_K2 * speed_rpm[i] * speed_rpm[i] - target_power + N630_POWER_CONSTANT;
            disc = b * b - 4.0f * a * c;
            if (disc < 0.0f)
                disc = 0.0f;

            if (current_out[i] >= 0.0f)
                current_out[i] = (-b + sqrtf(disc)) / (2.0f * a);
            else
                current_out[i] = (-b - sqrtf(disc)) / (2.0f * a);
        }
    }
}

N630MotorInstance *N630MotorInit(Motor_Init_Config_s *config)
{
    N630MotorInstance *instance;
    uint32_t controller_id;

    if (config == NULL || config->can_init_config.can_handle == NULL)
        return NULL;

    if (s_motor_count >= N630_MOTOR_CNT)
        return NULL;

    instance = (N630MotorInstance *)malloc(sizeof(N630MotorInstance));
    if (instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(N630MotorInstance));

    instance->motor_type = config->motor_type;
    instance->motor_settings = config->controller_setting_init_config;
    instance->motor_settings.motor_reverse_flag = N630_DEFAULT_MOTOR_REVERSE;
    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;

    controller_id = config->can_init_config.tx_id;
    config->can_init_config.id_type = CAN_ID_EXT;
    config->can_init_config.tx_id = n630_pack_can_id((uint8_t)controller_id, N630_CAN_PACKET_SET_CURRENT);
    config->can_init_config.rx_id = n630_pack_can_id((uint8_t)controller_id, N630_CAN_PACKET_STATUS);
    config->can_init_config.can_module_callback = N630MotorDecode;
    config->can_init_config.id = instance;
    instance->motor_can_instance = CANRegister(&config->can_init_config);
    if (instance->motor_can_instance == NULL)
    {
        free(instance);
        return NULL;
    }
    CANSetDLC(instance->motor_can_instance, 4);

    instance->stop_flag = MOTOR_ENALBED;
    instance->measure.online = 0;

    {
        Daemon_Init_Config_s daemon_config = {
            .callback = N630MotorLostCallback,
            .owner_id = instance,
            .reload_count = 2,
        };
        instance->daemon = DaemonRegister(&daemon_config);
    }

    n630_register_instance(instance);
    return instance;
}

void N630MotorChangeFeed(N630MotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (motor == NULL)
        return;

    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
}

void N630MotorStop(N630MotorInstance *motor)
{
    if (motor == NULL)
        return;
    motor->stop_flag = MOTOR_STOP;
}

void N630MotorEnable(N630MotorInstance *motor)
{
    if (motor == NULL)
        return;
    motor->stop_flag = MOTOR_ENALBED;
}

void N630MotorOuterLoop(N630MotorInstance *motor, Closeloop_Type_e outer_loop)
{
    if (motor == NULL)
        return;
    motor->motor_settings.outer_loop_type = outer_loop;
}

void N630MotorSetRef(N630MotorInstance *motor, float ref)
{
    if (motor == NULL)
        return;
    motor->motor_controller.pid_ref = ref;
}

void N630MotorControl(void)
{
    N630MotorInstance *active_motor[N630_MOTOR_CNT];
    float active_output[N630_MOTOR_CNT];
    uint8_t active_count = 0;

    for (uint8_t i = 0; i < s_motor_count; ++i)
    {
        N630MotorInstance *motor = s_motor_pool[i];
        Motor_Control_Setting_s *motor_setting;
        Motor_Controller_s *motor_controller;
        N630_Motor_Measure_s *measure;
        uint8_t feedback_ready;
        float pid_ref;
        float pid_measure;

        if (motor == NULL || motor->motor_can_instance == NULL)
            continue;

        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        feedback_ready = measure->online && (!motor->daemon || DaemonIsOnline(motor->daemon));
        pid_ref = motor_controller->pid_ref;

        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref = -pid_ref;

        if (feedback_ready && (motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = n630_read_ptr(motor_controller->other_angle_feedback_ptr, measure->total_angle);
            else
                pid_measure = measure->total_angle;
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += n630_read_ptr(motor_controller->speed_feedforward_ptr, 0.0f);

            if (feedback_ready)
            {
                if (motor_setting->speed_feedback_source == OTHER_FEED)
                    pid_measure = n630_read_ptr(motor_controller->other_speed_feedback_ptr, measure->speed_aps);
                else
                    pid_measure = measure->speed_aps;
            }
            else
            {
                pid_measure = 0.0f;
            }
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += n630_read_ptr(motor_controller->current_feedforward_ptr, 0.0f);

        if (feedback_ready && (motor_setting->close_loop_type & CURRENT_LOOP))
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->real_current, pid_ref);

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref = -pid_ref;

        if (motor->stop_flag == MOTOR_STOP)
            pid_ref = 0.0f;

        active_motor[active_count] = motor;
        active_output[active_count] = pid_ref;
        active_count++;
    }

    n630_apply_power_limit(active_motor, active_output, active_count);

    for (uint8_t i = 0; i < active_count; ++i)
        n630_send_current(active_motor[i], active_output[i]);
}
