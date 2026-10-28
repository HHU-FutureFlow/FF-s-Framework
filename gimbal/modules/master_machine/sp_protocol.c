/**
 ******************************************************************************
 * @file    sp_protocol.c
 * @author  HNU YueLu EC
 * @brief   同济视觉 SP 协议串口模块实现(经 MicroUSB 虚拟串口 VCP)
 *          按《FF步兵SP自瞄移植改动清单》从 HDRM 移植到 FF-s-Framework 云台板
 * @attention
 *  - 帧结构/CRC16 与视觉端 tongjiceshi io::Gimbal 完全对齐(帧头'S''P'+crc16)
 *  - 接收在 USB 中断上下文(CDC_Receive_FS)触发, 回调内完成找帧头/凑帧/校验,
 *    避免数据被下一包覆盖; 帧可能被拆包/粘包, 采用逐字节状态机
 *  - 数据源由 HDRM 的 QEKF_INS 改为 FF 的 INS(ins_task.h 已开放 extern),
 *    反馈源全部来自云台 C 板; pitch 使用 INS.Pitch(FF 角度环反馈同源, 区别于
 *    HDRM 的 Roll 安装适配)
 ******************************************************************************
 */
#include "sp_protocol.h"

#include "bsp_usb.h"
#include "bsp_log.h"
#include "daemon.h"
#include "ins_task.h"

/* ============================ CRC16(与视觉端一致) ============================ */
/* CRC16/MODBUS 查表法: init=0xffff, 字节流小端存放, 计算范围不含 crc16 字段 */
static const uint16_t SP_CRC16_TABLE[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
  0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
  0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
  0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
  0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
  0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
  0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
  0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
  0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
  0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
  0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
  0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

static uint16_t SPGetCRC16(const uint8_t *data, uint32_t len)
{
    uint16_t crc16 = 0xffff;
    uint8_t byte, i;
    while (len--)
    {
        byte = *data++;
        i = (uint8_t)((crc16 ^ byte) & 0x00ff);
        crc16 = (uint16_t)((crc16 >> 8) ^ SP_CRC16_TABLE[i]);
    }
    return crc16;
}

/* 视觉端校验: 帧内 crc16 小端存放(低字节在前) */
static uint8_t SPCheckCRC16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = (uint16_t)((data[len - 1] << 8) | data[len - 2]);
    return SPGetCRC16(data, len - 2) == crc;
}

/* ============================ 发送(GimbalToVision) ============================ */
static uint8_t sp_send_mode = 0;       // 上报模式
static float sp_bullet_speed = 15.0f;  // 弹速 m/s
static uint16_t sp_bullet_count = 0;   // 累计发弹数

void SPProtocolSetMode(uint8_t mode) { sp_send_mode = mode; }
void SPProtocolSetBullet(float speed, uint16_t count)
{
    sp_bullet_speed = speed;
    sp_bullet_count = count;
}

void SPProtocolSend(void)
{
    static SP_GimbalToVision_t tx;
    tx.head[0] = 'S';
    tx.head[1] = 'P';
    tx.mode = sp_send_mode;

    /* 四元数: INS.q 顺序即 [w,x,y,z], 与视觉端按 wxyz 读取一致, 直接填入
     * 反馈源全部来自云台 C 板(FF 云台板 INS) */
    tx.q[0] = INS.q[0];
    tx.q[1] = INS.q[1];
    tx.q[2] = INS.q[2];
    tx.q[3] = INS.q[3];

    /* 云台角度/角速度(rad, rad/s):
     * INS.Yaw/Pitch 单位为度, 换算为弧度; Gyro 为 rad/s(轴约定:
     * Gyro[2]=yaw 轴角速度, Gyro[1]=pitch 轴角速度).
     * pitch 使用 INS.Pitch 与 FF 角度环(IMU Pitch)反馈同源 */
    tx.yaw = INS.Yaw * 0.01745329252f;
    tx.yaw_vel = INS.Gyro[2];
    tx.pitch = INS.Pitch * 0.01745329252f;
    tx.pitch_vel = INS.Gyro[1];

    tx.bullet_speed = sp_bullet_speed;
    tx.bullet_count = sp_bullet_count;

    tx.crc16 = SPGetCRC16((uint8_t *)&tx, sizeof(tx) - sizeof(tx.crc16));
    USBTransmit((uint8_t *)&tx, sizeof(tx));
}

/* ============================ 接收(VisionToGimbal) ============================ */
static SP_VisionToGimbal_t sp_rx;      // 校验通过后更新的最新一帧
static uint8_t sp_rx_valid = 0;        // 是否已收到过有效帧
static DaemonInstance *sp_daemon;      // 视觉离线检测
static uint8_t *sp_usb_rx_buff_src;    // USB 接收缓冲指针(USBInit 返回)

static uint8_t sp_rx_buff[sizeof(SP_VisionToGimbal_t)]; // 帧拼装缓冲
static uint16_t sp_rx_idx = 0;                          // 已拼装字节数

/* USB 中断上下文调用: 逐字节找 'S','P' 帧头, 凑满整帧后校验 */
static void SPParseVision(uint16_t recv_len)
{
    uint8_t *src = sp_usb_rx_buff_src;
    uint16_t i, j;
    for (i = 0; i < recv_len; i++)
    {
        uint8_t b = src[i];
        if (sp_rx_idx < 2)
        {
            /* 找帧头 */
            if (sp_rx_idx == 0)
            {
                if (b == 'S') sp_rx_buff[0] = b, sp_rx_idx = 1;
            }
            else /* sp_rx_idx == 1 */
            {
                if (b == 'P') sp_rx_buff[1] = b, sp_rx_idx = 2;
                else          sp_rx_idx = 0; /* 重新找头 */
            }
        }
        else
        {
            sp_rx_buff[sp_rx_idx++] = b;
            if (sp_rx_idx == sizeof(sp_rx_buff))
            {
                sp_rx_idx = 0;
                if (SPCheckCRC16(sp_rx_buff, sizeof(sp_rx_buff)))
                {
                    /* 校验通过, 更新最新帧(小端机器上 packed 结构字节序天然一致) */
                    for (j = 0; j < sizeof(sp_rx); j++)
                        ((uint8_t *)&sp_rx)[j] = sp_rx_buff[j];
                    sp_rx_valid = 1;
                    DaemonReload(sp_daemon);
                }
            }
        }
    }
}

const SP_VisionToGimbal_t *SPProtocolGetRecv(void)
{
    return sp_rx_valid ? &sp_rx : NULL;
}

/* 视觉在线状态查询: 距上次有效帧 25ms 内(sp_daemon reload_count=5, 由 DaemonTask 递减)视为在线,
 * 供消费口(VisionAimSet)在视觉掉线后停止按旧帧增量累加.
 * Init 前 sp_daemon 为空指针时保守按"离线"处理, 避免异常时序下消费旧增量导致云台漂移. */
uint8_t SPProtocolIsOnline(void)
{
    return (sp_daemon != NULL) && DaemonIsOnline(sp_daemon);
}

void SPProtocolInit(void)
{
    USB_Init_Config_s conf = {.rx_cbk = SPParseVision};
    sp_usb_rx_buff_src = USBInit(conf); // 获取接收缓冲指针(USBInit 返回 UserRxBufferFS)

    Daemon_Init_Config_s daemon_conf = {
        .callback = NULL,      // USB 接收为硬件自动连续, 无需离线重启
        .owner_id = NULL,
        .reload_count = 5,     // 5ms * 5 = 25ms 未收到有效帧判离线(由 DaemonTask 检查)
    };
    sp_daemon = DaemonRegister(&daemon_conf);
    LOGINFO("[SP] SP protocol init success");
}
