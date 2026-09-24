#include "pc_comm.h"

#include "bsp_usb.h"
#include "crc8.h"
#include "seasky_protocol.h"

#include <string.h>

#define PC_COMM_MAX_HANDLERS 8u
#define PC_COMM_NAVIGATION_COMMAND_HEADER 0xA5u
#define PC_COMM_NAVIGATION_COMMAND_SIZE 15u
#define PC_COMM_NAVIGATION_YAW_HEADER 0xA9u
#define PC_COMM_NAVIGATION_YAW_SIZE 7u

typedef struct
{
    uint16_t command_id;
    PCCommFrameHandler handler;
} PCCommHandlerEntry_s;

static uint8_t *pc_comm_rx_buffer;
static uint8_t pc_comm_frame[PC_COMM_MAX_FRAME_SIZE];
static uint16_t pc_comm_frame_length;
static uint8_t pc_comm_initialized;
static volatile uint8_t pc_comm_tx_busy;
static uint8_t pc_comm_tx_buffer[PC_COMM_MAX_FRAME_SIZE];
static PCCommHandlerEntry_s pc_comm_handlers[PC_COMM_MAX_HANDLERS];
static PCCommRawFrameHandler pc_comm_navigation_handler;
volatile PCCommDebug_s pc_comm_debug;

static void PCCommTxCompleteCallback(uint16_t length)
{
    (void)length;
    pc_comm_tx_busy = 0u;
}

static void PCCommResetParser(void)
{
    pc_comm_frame_length = 0u;
}

static uint8_t PCCommNavigationChecksumValid(void)
{
    uint16_t checksum = 0u;
    uint16_t received_checksum;

    /* A5 与 A9 是与 Seasky 帧共用 CDC 字节流的固定长度私有帧。 */
    if (pc_comm_frame_length == PC_COMM_NAVIGATION_COMMAND_SIZE)
    {
        for (uint16_t i = 0u; i < 13u; i++)
        {
            checksum = (uint16_t)(checksum + pc_comm_frame[i]);
        }
        received_checksum = (uint16_t)pc_comm_frame[13] |
                            ((uint16_t)pc_comm_frame[14] << 8u);
    }
    else if (pc_comm_frame_length == PC_COMM_NAVIGATION_YAW_SIZE)
    {
        for (uint16_t i = 0u; i < 5u; i++)
        {
            checksum = (uint16_t)(checksum + pc_comm_frame[i]);
        }
        received_checksum = (uint16_t)pc_comm_frame[5] |
                            ((uint16_t)pc_comm_frame[6] << 8u);
    }
    else
    {
        return 0u;
    }
    return (uint8_t)(checksum == received_checksum);
}

static uint8_t PCCommIsFrameStart(uint8_t byte)
{
    /* Seasky 帧以 PROTOCOL_CMD_ID 开始；A5 与 A9 是导航私有帧的起始字节。 */
    return (uint8_t)((byte == PROTOCOL_CMD_ID) ||
                     (byte == PC_COMM_NAVIGATION_YAW_HEADER));
}

static void PCCommResyncParser(void)
{
    for (uint16_t i = 1u; i < pc_comm_frame_length; i++)
    {
        if (PCCommIsFrameStart(pc_comm_frame[i]) != 0u)
        {
            uint16_t remaining = (uint16_t)(pc_comm_frame_length - i);
            memmove(pc_comm_frame, pc_comm_frame + i, remaining);
            pc_comm_frame_length = remaining;
            return;
        }
    }
    PCCommResetParser();
}

static void PCCommDispatchFrame(void)
{
    uint16_t flags = 0u;
    uint8_t payload[PC_COMM_MAX_FRAME_SIZE];
    uint16_t command_id;
    uint16_t data_length;
    uint16_t payload_length;

    data_length = (uint16_t)pc_comm_frame[1] |
                  ((uint16_t)pc_comm_frame[2] << 8u);
    if ((data_length < 2u) || ((uint16_t)(data_length + 8u) != pc_comm_frame_length))
    {
        pc_comm_debug.rx_invalid_length_count++;
        return;
    }

    command_id = get_protocol_info(pc_comm_frame, &flags, payload);
    if (command_id == 0u)
    {
        pc_comm_debug.rx_invalid_frame_count++;
        return;
    }

    pc_comm_debug.rx_frame_count++;

    payload_length = (uint16_t)(data_length - 2u);
    for (uint8_t i = 0u; i < PC_COMM_MAX_HANDLERS; i++)
    {
        if ((pc_comm_handlers[i].handler != NULL) &&
            (pc_comm_handlers[i].command_id == command_id))
        {
            pc_comm_handlers[i].handler(command_id, flags, payload, payload_length);
            return;
        }
    }

    pc_comm_debug.rx_unhandled_frame_count++;
}

static void PCCommFeedByte(uint8_t byte)
{
    uint16_t expected_length = 0u;
    uint8_t legacy_header_valid = 0u;

    if (pc_comm_frame_length == 0u)
    {
        if (PCCommIsFrameStart(byte) == 0u)
        {
            return;
        }
    }

    if (pc_comm_frame_length >= PC_COMM_MAX_FRAME_SIZE)
    {
        PCCommResetParser();
        if (PCCommIsFrameStart(byte) != 0u)
        {
            pc_comm_frame[pc_comm_frame_length++] = byte;
        }
        return;
    }

    pc_comm_frame[pc_comm_frame_length++] = byte;

    if (pc_comm_frame[0] == PC_COMM_NAVIGATION_YAW_HEADER)
    {
        /* A9 长度仅 7 字节，必须在按 Seasky 长度字段解释前优先完成解析。 */
        if (pc_comm_frame_length < PC_COMM_NAVIGATION_YAW_SIZE)
        {
            return;
        }

        if (PCCommNavigationChecksumValid() != 0u)
        {
            if (pc_comm_navigation_handler != NULL)
            {
                pc_comm_navigation_handler(pc_comm_frame, pc_comm_frame_length);
                pc_comm_debug.rx_frame_count++;
            }
            else
            {
                pc_comm_debug.rx_unhandled_frame_count++;
            }
            PCCommResetParser();
            return;
        }

        pc_comm_debug.rx_invalid_frame_count++;
        PCCommResyncParser();
        return;
    }

    if (pc_comm_frame_length < 4u)
    {
        return;
    }

    expected_length = (uint16_t)pc_comm_frame[1] |
                      ((uint16_t)pc_comm_frame[2] << 8u);
    expected_length = (uint16_t)(expected_length + 8u);
    legacy_header_valid = (uint8_t)((crc_8(pc_comm_frame, 3u) == pc_comm_frame[3]) &&
                                    (expected_length >= 10u) &&
                                    (expected_length <= PC_COMM_MAX_FRAME_SIZE));

    if ((pc_comm_frame_length == PC_COMM_NAVIGATION_COMMAND_SIZE) && PCCommNavigationChecksumValid())
    {
        /* A5 的首字节也等于 Seasky SOF；在第 15 字节用私有校验和确认其身份。 */
        if (pc_comm_navigation_handler != NULL)
        {
            pc_comm_navigation_handler(pc_comm_frame, pc_comm_frame_length);
            pc_comm_debug.rx_frame_count++;
        }
        else
        {
            pc_comm_debug.rx_unhandled_frame_count++;
        }
        PCCommResetParser();
        return;
    }

    if ((legacy_header_valid != 0u) && (pc_comm_frame_length == expected_length))
    {
        PCCommDispatchFrame();
        PCCommResetParser();
        return;
    }

    if ((pc_comm_frame_length < PC_COMM_NAVIGATION_COMMAND_SIZE) ||
        ((legacy_header_valid != 0u) && (pc_comm_frame_length < expected_length)))
    {
        return;
    }

    pc_comm_debug.rx_invalid_frame_count++;
    if ((expected_length < 10u) || (expected_length > PC_COMM_MAX_FRAME_SIZE))
    {
        pc_comm_debug.rx_invalid_length_count++;
    }
    PCCommResyncParser();
}

static void PCCommRxCallback(uint16_t received_length)
{
    if (pc_comm_rx_buffer == NULL)
    {
        return;
    }

    for (uint16_t i = 0u; i < received_length; i++)
    {
        PCCommFeedByte(pc_comm_rx_buffer[i]);
    }
}

void PCCommInit(void)
{
    USB_Init_Config_s usb_config = {
        .tx_cbk = PCCommTxCompleteCallback,
        .rx_cbk = PCCommRxCallback,
    };

    if (pc_comm_initialized != 0u)
    {
        return;
    }

    memset(pc_comm_handlers, 0, sizeof(pc_comm_handlers));
    pc_comm_navigation_handler = NULL;
    memset((void *)&pc_comm_debug, 0, sizeof(pc_comm_debug));
    PCCommResetParser();
    pc_comm_tx_busy = 0u;
    pc_comm_rx_buffer = USBInit(usb_config);
    pc_comm_initialized = 1u;
}

uint8_t PCCommRegisterNavigationHandler(PCCommRawFrameHandler handler)
{
    if (handler == NULL)
    {
        return 0u;
    }
    pc_comm_navigation_handler = handler;
    return 1u;
}

PCCommSendStatus_e PCCommSendBytes(const uint8_t *data, uint16_t length)
{
    uint32_t primask;
    uint8_t result;

    if ((pc_comm_initialized == 0u) || (data == NULL) ||
        (length == 0u) || (length > PC_COMM_MAX_FRAME_SIZE))
    {
        return PC_COMM_SEND_ERROR;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (pc_comm_tx_busy != 0u)
    {
        if (primask == 0u)
        {
            __enable_irq();
        }
        pc_comm_debug.tx_busy_count++;
        return PC_COMM_SEND_BUSY;
    }
    pc_comm_tx_busy = 1u;
    memcpy(pc_comm_tx_buffer, data, length);
    if (primask == 0u)
    {
        __enable_irq();
    }

    result = USBTransmit(pc_comm_tx_buffer, length);
    if (result != USBD_OK)
    {
        pc_comm_tx_busy = 0u;
        pc_comm_debug.tx_busy_count++;
    }
    else
    {
        pc_comm_debug.tx_frame_count++;
    }
    if (result == USBD_OK)
    {
        return PC_COMM_SEND_OK;
    }
    if (result == USBD_BUSY)
    {
        return PC_COMM_SEND_BUSY;
    }
    return PC_COMM_SEND_ERROR;
}

uint8_t PCCommRegisterHandler(uint16_t command_id, PCCommFrameHandler handler)
{
    if (handler == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < PC_COMM_MAX_HANDLERS; i++)
    {
        if ((pc_comm_handlers[i].handler != NULL) &&
            (pc_comm_handlers[i].command_id == command_id))
        {
            pc_comm_handlers[i].handler = handler;
            return 1u;
        }
    }

    for (uint8_t i = 0u; i < PC_COMM_MAX_HANDLERS; i++)
    {
        if (pc_comm_handlers[i].handler == NULL)
        {
            pc_comm_handlers[i].command_id = command_id;
            pc_comm_handlers[i].handler = handler;
            return 1u;
        }
    }

    return 0u;
}

PCCommSendStatus_e PCCommSendFloats(uint16_t command_id,
                                    uint16_t flags,
                                    const float *values,
                                    uint8_t value_count)
{
    uint16_t frame_length;

    if ((pc_comm_initialized == 0u) || (values == NULL) ||
        (value_count == 0u) || (value_count > 12u))
    {
        return PC_COMM_SEND_ERROR;
    }

    {
        uint8_t frame[PC_COMM_MAX_FRAME_SIZE];

        get_protocol_send_data(command_id, flags, (float *)values,
                               value_count, frame, &frame_length);
        return PCCommSendBytes(frame, frame_length);
    }
}
