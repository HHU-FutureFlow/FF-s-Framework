/**
 * @file vt_link.h
 * @brief VT02 图传链路 0x0304 键鼠遥控数据接口
 */
#ifndef VT_LINK_H
#define VT_LINK_H

#include "stdint.h"
#include "usart.h"
#include "remote_control.h"

#define VT_LINK_RX_BUFF_SIZE 128u
#define VT_LINK_CMD_KEYMOUSE 0x0304u
#define VT_LINK_KEYMOUSE_DATA_LEN 12u
#define VT_LINK_OFFLINE_MS 150u

RC_ctrl_t *VTLinkInit(UART_HandleTypeDef *vt_usart_handle);
uint8_t VTLinkIsOnline(void);
RC_ctrl_t *VTLinkGetData(void);

extern volatile uint8_t vt_link_raw_buff[32];
extern volatile uint32_t vt_link_rx_event_cnt;
extern volatile uint32_t vt_link_frame_cnt;
extern volatile uint32_t vt_link_crc8_err_cnt;
extern volatile uint32_t vt_link_crc16_err_cnt;
extern volatile uint16_t vt_link_last_cmd_id;
extern volatile uint8_t vt_link_online;

#endif
