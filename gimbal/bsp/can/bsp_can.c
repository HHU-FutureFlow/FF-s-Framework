#include "bsp_can.h"
#include "main.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx;

static uint32_t CANPackFilterWord(uint32_t id, uint32_t id_type, uint32_t rtr)
{
    if (id_type == CAN_ID_EXT)
        return (id << 3) | CAN_ID_EXT | rtr;
    return (id << 21) | CAN_ID_STD | rtr;
}

static void CANAddFilter(CANInstance *_instance)
{
    CAN_FilterTypeDef can_filter_conf = {0};
    static uint8_t can1_filter_idx = 0;
    static uint8_t can2_filter_idx = 14;

#if CAN_FILTER_DEBUG_OPEN
    can_filter_conf.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_conf.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_conf.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_conf.SlaveStartFilterBank = 14;
    can_filter_conf.FilterBank = (_instance->can_handle == &hcan1) ? can1_filter_idx++ : can2_filter_idx++;
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;
    can_filter_conf.FilterIdHigh = 0x0000U;
    can_filter_conf.FilterIdLow = 0x0000U;
    can_filter_conf.FilterMaskIdHigh = 0x0000U;
    can_filter_conf.FilterMaskIdLow = 0x0000U;
#else
    can_filter_conf.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_conf.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_conf.FilterFIFOAssignment = (_instance->tx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    can_filter_conf.SlaveStartFilterBank = 14;
    can_filter_conf.FilterBank = (_instance->can_handle == &hcan1) ? can1_filter_idx++ : can2_filter_idx++;
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;
   if (_instance->id_type == CAN_ID_STD)
    {
        can_filter_conf.FilterIdHigh = (uint16_t)((_instance->rx_id & 0x7FFU) << 5);
        can_filter_conf.FilterIdLow = 0x0000U;
        can_filter_conf.FilterMaskIdHigh = 0xFFE0U;
        can_filter_conf.FilterMaskIdLow = 0x0000U;
    }
    else
    {
        uint32_t filter_word = CANPackFilterWord(_instance->rx_id, _instance->id_type, CAN_RTR_DATA);
        can_filter_conf.FilterIdHigh = (uint16_t)((filter_word & 0xFFFF0000U) >> 16);
        can_filter_conf.FilterIdLow = (uint16_t)(filter_word & 0x0000FFFFU);
        can_filter_conf.FilterMaskIdHigh = 0xFFFFU;
        can_filter_conf.FilterMaskIdLow = 0xFFFEU;
    }
#endif
    HAL_CAN_ConfigFilter(_instance->can_handle, &can_filter_conf);
}

static void CANServiceInit(void)
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    CANInstance *instance;

    if (!idx)
    {
        CANServiceInit();
        LOGINFO("[bsp_can] CAN Service Init");
    }

    if (idx >= CAN_MX_REGISTER_CNT)
    {
        while (1)
            LOGERROR("[bsp_can] CAN instance exceeded MAX num, consider balance the load of CAN bus");
    }

    for (size_t i = 0; i < idx; i++)
    {
        if (can_instance[i]->rx_id == config->rx_id &&
            can_instance[i]->can_handle == config->can_handle &&
            can_instance[i]->id_type == config->id_type)
        {
            while (1)
                LOGERROR("[bsp_can] CAN id crash, tx [%d] or rx [%d] already registered", &config->tx_id, &config->rx_id);
        }
    }

    instance = (CANInstance *)malloc(sizeof(CANInstance));
    if (instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(CANInstance));

    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;
    instance->id_type = config->id_type;

    instance->txconf.IDE = config->id_type;
    instance->txconf.RTR = CAN_RTR_DATA;
    instance->txconf.DLC = 0x08;
    if (config->id_type == CAN_ID_EXT)
        instance->txconf.ExtId = config->tx_id;
    else
        instance->txconf.StdId = config->tx_id;

    CANAddFilter(instance);
    can_instance[idx++] = instance;
    return instance;
}

uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    static uint32_t busy_count;
    static volatile float wait_time __attribute__((unused));
    float dwt_start;

    if (_instance == NULL)
        return 0;

    dwt_start = DWT_GetTimeline_ms();
    while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0)
    {
        if (DWT_GetTimeline_ms() - dwt_start > timeout)
        {
            LOGWARNING("[bsp_can] CAN MAILbox full! failed to add msg to mailbox. Cnt [%d]", busy_count);
            busy_count++;
            return 0;
        }
    }

    wait_time = DWT_GetTimeline_ms() - dwt_start;
    if (HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, _instance->tx_buff, &_instance->tx_mailbox))
    {
        LOGWARNING("[bsp_can] CAN bus BUS! cnt:%d", busy_count);
        busy_count++;
        return 0;
    }
    return 1;
}

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    if (length > 8 || length == 0)
        while (1)
            LOGERROR("[bsp_can] CAN DLC error! check your code or wild pointer");
    _instance->txconf.DLC = length;
}

static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf;
    uint8_t can_rx_buff[8];

    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox))
    {
        HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff);

        if (_hcan == &hcan1 && rxconf.IDE == CAN_ID_EXT)
        {
            LOGINFO("[bsp_can] CAN1 EXT rx id=0x%08lX dlc=%lu data=%02X %02X %02X %02X %02X %02X %02X %02X",
                    rxconf.ExtId,
                    rxconf.DLC,
                    can_rx_buff[0], can_rx_buff[1], can_rx_buff[2], can_rx_buff[3],
                    can_rx_buff[4], can_rx_buff[5], can_rx_buff[6], can_rx_buff[7]);
        }

        for (size_t i = 0; i < idx; ++i)
        {
            if (_hcan == can_instance[i]->can_handle && rxconf.IDE == can_instance[i]->id_type)
            {
                if ((rxconf.IDE == CAN_ID_STD && rxconf.StdId == can_instance[i]->rx_id) ||
                    (rxconf.IDE == CAN_ID_EXT && rxconf.ExtId == can_instance[i]->rx_id))
                {
                    if (can_instance[i]->can_module_callback != NULL)
                    {
                        can_instance[i]->rx_len = rxconf.DLC;
                        memcpy(can_instance[i]->rx_buff, can_rx_buff, rxconf.DLC);
                        can_instance[i]->can_module_callback(can_instance[i]);
                    }
                    return;
                }
            }
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1);
}
