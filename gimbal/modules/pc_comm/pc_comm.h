#ifndef PC_COMM_H
#define PC_COMM_H

#include <stdint.h>

#define PC_COMM_CMD_VISION_COMMAND       0x0001u
#define PC_COMM_CMD_VISION_FEEDBACK      0x0002u
#define PC_COMM_CMD_NAVIGATION_COMMAND   0x0101u
#define PC_COMM_CMD_NAVIGATION_FEEDBACK  0x0102u

#define PC_COMM_MAX_FRAME_SIZE 64u

typedef enum
{
    PC_COMM_SEND_OK = 0u,
    PC_COMM_SEND_BUSY = 1u,
    PC_COMM_SEND_ERROR = 2u,
} PCCommSendStatus_e;

typedef void (*PCCommFrameHandler)(uint16_t command_id,
                                   uint16_t flags,
                                   const uint8_t *payload,
                                   uint16_t payload_length);
typedef void (*PCCommRawFrameHandler)(const uint8_t *frame, uint16_t frame_length);

typedef struct
{
    uint32_t rx_frame_count;
    uint32_t rx_invalid_frame_count;
    uint32_t rx_invalid_length_count;
    uint32_t rx_unhandled_frame_count;
    uint32_t tx_frame_count;
    uint32_t tx_busy_count;
} PCCommDebug_s;

extern volatile PCCommDebug_s pc_comm_debug;

void PCCommInit(void);
uint8_t PCCommRegisterHandler(uint16_t command_id, PCCommFrameHandler handler);
uint8_t PCCommRegisterNavigationHandler(PCCommRawFrameHandler handler);
PCCommSendStatus_e PCCommSendBytes(const uint8_t *data, uint16_t length);
PCCommSendStatus_e PCCommSendFloats(uint16_t command_id,
                                    uint16_t flags,
                                    const float *values,
                                    uint8_t value_count);

#endif
