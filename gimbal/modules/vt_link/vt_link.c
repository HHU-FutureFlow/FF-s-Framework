/**
 * @file vt_link.c
 * @brief VT02 图传链路 0x0304 键鼠数据解析
 */
#include "vt_link.h"
#include "referee_protocol.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "bsp_log.h"
#include "string.h"

#define VT_LINK_STREAM_BUFF_SIZE 256u
#define VT_LINK_MAX_DATA_LEN 127u
#define VT_LINK_RAW_DUMP_LEN 32u

static RC_ctrl_t vt_ctrl[2];
static USARTInstance *vt_usart_instance;
static DaemonInstance *vt_daemon_instance;
static uint8_t vt_init_flag;
static uint8_t vt_resync_pending;
static uint8_t vt_stream_buff[VT_LINK_STREAM_BUFF_SIZE];
static uint16_t vt_stream_len;

volatile uint8_t vt_link_raw_buff[VT_LINK_RAW_DUMP_LEN];
volatile uint32_t vt_link_rx_event_cnt;
volatile uint32_t vt_link_frame_cnt;
volatile uint32_t vt_link_crc8_err_cnt;
volatile uint32_t vt_link_crc16_err_cnt;
volatile uint16_t vt_link_last_cmd_id;
volatile uint8_t vt_link_online;

volatile uint32_t vt_debug_rx_event_cnt;
volatile uint16_t vt_debug_last_recv_len;
volatile uint32_t vt_debug_frame_cnt;
volatile uint32_t vt_debug_crc8_err_cnt;
volatile uint32_t vt_debug_crc16_err_cnt;
volatile uint16_t vt_debug_last_cmd_id;
volatile uint16_t vt_debug_last_data_len;
volatile uint16_t vt_debug_stream_len;
volatile uint8_t vt_debug_online;
volatile uint8_t vt_debug_raw_len;
volatile uint32_t vt_debug_raw_word0;
volatile uint32_t vt_debug_raw_word1;

static void VTLinkParseKeyMouse(const uint8_t *data)
{
    uint16_t key_now;
    uint16_t key_last;
    uint16_t key_with_ctrl;
    uint16_t key_with_shift;
    uint16_t key_last_with_ctrl;
    uint16_t key_last_with_shift;

    vt_ctrl[TEMP].mouse.x = (int16_t)(data[0] | (data[1] << 8));
    vt_ctrl[TEMP].mouse.y = (int16_t)(data[2] | (data[3] << 8));
    vt_ctrl[TEMP].mouse.press_l = data[6] ? 1u : 0u;
    vt_ctrl[TEMP].mouse.press_r = data[7] ? 1u : 0u;
    *(uint16_t *)&vt_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(data[8] | (data[9] << 8));

    if (vt_ctrl[TEMP].key[KEY_PRESS].ctrl)
        vt_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = vt_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&vt_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));

    if (vt_ctrl[TEMP].key[KEY_PRESS].shift)
        vt_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = vt_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&vt_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    key_now = vt_ctrl[TEMP].key[KEY_PRESS].keys;
    key_last = vt_ctrl[LAST].key[KEY_PRESS].keys;
    key_with_ctrl = vt_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys;
    key_with_shift = vt_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys;
    key_last_with_ctrl = vt_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys;
    key_last_with_shift = vt_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys;

    if (vt_resync_pending)
    {
        memcpy(&vt_ctrl[LAST], &vt_ctrl[TEMP], sizeof(RC_ctrl_t));
        vt_resync_pending = 0u;
        key_last = key_now;
        key_last_with_ctrl = key_with_ctrl;
        key_last_with_shift = key_with_shift;
    }

    for (uint16_t i = 0, mask = 1u; i < 16u; ++i, mask <<= 1u)
    {
        if (i == 4u || i == 5u)
            continue;

        if ((key_now & mask) && !(key_last & mask) &&
            !(key_with_ctrl & mask) && !(key_with_shift & mask))
            vt_ctrl[TEMP].key_count[KEY_PRESS][i]++;

        if ((key_with_ctrl & mask) && !(key_last_with_ctrl & mask))
            vt_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;

        if ((key_with_shift & mask) && !(key_last_with_shift & mask))
            vt_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&vt_ctrl[LAST], &vt_ctrl[TEMP], sizeof(RC_ctrl_t));
}

static void VTLinkParseStream(void)
{
    uint16_t offset = 0u;

    while ((uint32_t)offset + LEN_HEADER <= vt_stream_len)
    {
        uint16_t data_len;
        uint16_t frame_len;
        uint16_t cmd_id;

        if (vt_stream_buff[offset + SOF] != REFEREE_SOF)
        {
            offset++;
            continue;
        }

        data_len = (uint16_t)(vt_stream_buff[offset + DATA_LENGTH] |
                              (vt_stream_buff[offset + DATA_LENGTH + 1u] << 8));
        vt_debug_last_data_len = data_len;
        if (data_len > VT_LINK_MAX_DATA_LEN)
        {
            offset++;
            continue;
        }

        frame_len = (uint16_t)(LEN_HEADER + LEN_CMDID + data_len + LEN_TAIL);
        if ((uint32_t)offset + frame_len > vt_stream_len)
            break;

        if (!Verify_CRC8_Check_Sum(vt_stream_buff + offset, LEN_HEADER))
        {
            vt_link_crc8_err_cnt++;
            vt_debug_crc8_err_cnt++;
            offset++;
            continue;
        }

        if (!Verify_CRC16_Check_Sum(vt_stream_buff + offset, frame_len))
        {
            vt_link_crc16_err_cnt++;
            vt_debug_crc16_err_cnt++;
            offset++;
            continue;
        }

        cmd_id = (uint16_t)(vt_stream_buff[offset + CMD_ID_Offset] |
                            (vt_stream_buff[offset + CMD_ID_Offset + 1u] << 8));
        vt_link_last_cmd_id = cmd_id;
        vt_debug_last_cmd_id = cmd_id;

        if (cmd_id == VT_LINK_CMD_KEYMOUSE && data_len == VT_LINK_KEYMOUSE_DATA_LEN)
        {
            VTLinkParseKeyMouse(vt_stream_buff + offset + DATA_Offset);
            vt_link_frame_cnt++;
            vt_debug_frame_cnt++;
            vt_link_online = 1u;
            vt_debug_online = 1u;
            DaemonReload(vt_daemon_instance);
        }

        offset = (uint16_t)(offset + frame_len);
    }

    if (offset > 0u)
    {
        memmove(vt_stream_buff, vt_stream_buff + offset, vt_stream_len - offset);
        vt_stream_len = (uint16_t)(vt_stream_len - offset);
    }

    vt_debug_stream_len = vt_stream_len;
}

static void VTLinkRxCallback(void)
{
    uint16_t recv_len = vt_usart_instance->recv_len;
    uint16_t dump_len = recv_len < VT_LINK_RAW_DUMP_LEN ? recv_len : VT_LINK_RAW_DUMP_LEN;

    vt_link_rx_event_cnt++;
    vt_debug_rx_event_cnt++;
    vt_debug_last_recv_len = recv_len;
    vt_debug_raw_len = (uint8_t)dump_len;
    vt_debug_raw_word0 = 0u;
    vt_debug_raw_word1 = 0u;
    for (uint16_t i = 0u; i < dump_len && i < 4u; ++i)
        vt_debug_raw_word0 |= (uint32_t)vt_usart_instance->recv_buff[i] << (8u * i);
    for (uint16_t i = 4u; i < dump_len && i < 8u; ++i)
        vt_debug_raw_word1 |= (uint32_t)vt_usart_instance->recv_buff[i] << (8u * (i - 4u));
    memcpy((void *)vt_link_raw_buff, vt_usart_instance->recv_buff, dump_len);

    if (recv_len == 0u)
        return;

    if ((uint32_t)vt_stream_len + recv_len > VT_LINK_STREAM_BUFF_SIZE)
        vt_stream_len = 0u;

    memcpy(vt_stream_buff + vt_stream_len, vt_usart_instance->recv_buff, recv_len);
    vt_stream_len = (uint16_t)(vt_stream_len + recv_len);
    VTLinkParseStream();
}

static void VTLinkLostCallback(void *id)
{
    (void)id;

    memset(&vt_ctrl[TEMP].mouse, 0, sizeof(vt_ctrl[TEMP].mouse));
    memset(&vt_ctrl[TEMP].key[KEY_PRESS], 0, sizeof(Key_t));
    memset(&vt_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    memset(&vt_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));
    memcpy(&vt_ctrl[LAST], &vt_ctrl[TEMP], sizeof(RC_ctrl_t));
    vt_link_online = 0u;
    vt_debug_online = 0u;
    vt_resync_pending = 1u;
    USARTServiceInit(vt_usart_instance);
    LOGWARNING("[vt_link] keymouse data lost, restart usart rx");
}

RC_ctrl_t *VTLinkInit(UART_HandleTypeDef *vt_usart_handle)
{
    USART_Init_Config_s usart_conf;
    Daemon_Init_Config_s daemon_conf;

    if (vt_usart_handle == NULL)
        return NULL;

    if (vt_init_flag)
        return vt_ctrl;

    if (vt_usart_handle->Instance != USART6)
        LOGWARNING("[vt_link] expected USART6 for VT02 input");
    if (vt_usart_handle->Init.BaudRate != 115200u)
        LOGWARNING("[vt_link] expected USART6 baudrate 115200");

    memset(vt_ctrl, 0, sizeof(vt_ctrl));
    memset(vt_stream_buff, 0, sizeof(vt_stream_buff));
    vt_stream_len = 0u;
    vt_resync_pending = 1u;

    usart_conf.recv_buff_size = VT_LINK_RX_BUFF_SIZE;
    usart_conf.usart_handle = vt_usart_handle;
    usart_conf.module_callback = VTLinkRxCallback;
    vt_usart_instance = USARTRegister(&usart_conf);

    daemon_conf.reload_count = VT_LINK_OFFLINE_MS / 10u;
    daemon_conf.callback = VTLinkLostCallback;
    daemon_conf.owner_id = NULL;
    vt_daemon_instance = DaemonRegister(&daemon_conf);

    vt_init_flag = 1u;
    LOGINFO("[vt_link] init done");
    return vt_ctrl;
}

uint8_t VTLinkIsOnline(void)
{
    return vt_init_flag ? vt_link_online : 0u;
}

RC_ctrl_t *VTLinkGetData(void)
{
    return vt_init_flag ? vt_ctrl : NULL;
}
