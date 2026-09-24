#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "can.h"

#define CAN_MX_REGISTER_CNT 16
#define MX_CAN_FILTER_CNT (2 * 14)
#define DEVICE_CAN_CNT 2

/* 调试时置 1，可让 CAN 过滤器全放行，便于查看实际收到的报文 */
#define CAN_FILTER_DEBUG_OPEN 1

#pragma pack(1)
typedef struct _
{
    CAN_HandleTypeDef *can_handle;
    CAN_TxHeaderTypeDef txconf;
    uint32_t tx_id;
    uint32_t tx_mailbox;
    uint8_t tx_buff[8];
    uint8_t rx_buff[8];
    uint32_t rx_id;
    uint8_t rx_len;
    void (*can_module_callback)(struct _ *);
    void *id;
    uint32_t id_type;
} CANInstance;
#pragma pack()

typedef struct
{
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    void (*can_module_callback)(CANInstance *);
    void *id;
    uint32_t id_type;
} CAN_Init_Config_s;

CANInstance *CANRegister(CAN_Init_Config_s *config);
void CANSetDLC(CANInstance *_instance, uint8_t length);
uint8_t CANTransmit(CANInstance *_instance, float timeout);

#endif
