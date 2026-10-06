/**
 * @file    Chassis_protocol.c
 * @brief   底盘串口协议实现 (0xBB 收指令 / 0xCD 发里程计与轮计数)
 *
 * 字节序说明:
 *   - 0xBB 帧的 v_linear / v_angular 是 int16 大端(高位在前), 对应
 *     chassis_bridge.py 里手工拼的 buf[1]=hi, buf[2]=lo。
 *   - 0xCD V2 帧全部使用小端，对应 Python 的 '<BHIfffii'。
 */

#include "Chassis_protocol.h"
#include "Chassis_task.h"
#include "bsp.h"          /* UART_Send_Data / huart2 / memcpy */

/* ======================== 私有变量 ======================== */

/* 速度指令超时停车: 超过 250ms 没收到合法 0xBB 帧, 就自动停车。
   这是安全机制 —— 上位机崩溃/断线时, 不能让轮子保持最后速度一直跑。
   Host_ApplyPendingCommand() 每 5ms 被调用一次(200Hz), 所以
   250ms = 50 个 tick。 */
#define HOST_CMD_TIMEOUT_MS     250u
#define HOST_CMD_TIMEOUT_TICKS  (HOST_CMD_TIMEOUT_MS / 5u)

/* 0xBB 帧解析状态机 */
static uint8_t s_host_rx_state = 0u;
static uint8_t s_host_rx_buf[7];
static uint8_t s_host_rx_idx = 0u;

/* 上位机指令缓存: UART 中断里写, Chassis_Update 里读。
   两个中断同为优先级 0 且不互相抢占, 所以这里顺序读写是安全的;
   volatile 防止编译器优化掉读/写。 */
static volatile float   s_host_v_mps      = 0.0f;
static volatile float   s_host_w_radps    = 0.0f;
static volatile uint8_t s_host_cmd_pending = 0u;

/* 超时停车状态 */
static uint32_t s_host_idle_ticks  = 0u;   /* 自上次指令以来的 5ms 计数 */
static uint8_t  s_host_ever_got_cmd = 0u;  /* 是否收到过至少一条指令 */
static uint8_t  s_host_stopped      = 0u;  /* 是否已因超时停车(只停一次) */

/* 里程计帧缓冲 (static: DMA 发送是异步的, 局部变量会失效) */
static uint8_t s_odom_buf[30];
static uint16_t s_odom_sequence;

/* TIM6 单调时基，单位 100 us。 */
extern volatile uint32_t nowtime;

/* ======================== 私有函数 ======================== */

/**
 * @brief  喂一个字节给 0xBB 帧状态机
 * @note   帧结构: BB v_hi v_lo w_hi w_lo cksum 55 (7 字节)
 */
static void Host_Input_Byte(uint8_t b)
{
    switch (s_host_rx_state) {
    case 0u:                      /* 等帧头 */
        if (b == HOST_HEAD_CMD) {
            s_host_rx_buf[0] = b;
            s_host_rx_idx = 1u;
            s_host_rx_state = 1u;
        }
        break;

    case 1u:                      /* 收剩余 6 字节 */
        s_host_rx_buf[s_host_rx_idx++] = b;
        if (s_host_rx_idx >= 7u) {
            s_host_rx_state = 0u;

            /* 校验帧尾 + 校验和 (buf[1..4] 求和) */
            if ((s_host_rx_buf[6] == HOST_FOOTER) &&
                (s_host_rx_buf[5] == (uint8_t)(s_host_rx_buf[1] +
                                                s_host_rx_buf[2] +
                                                s_host_rx_buf[3] +
                                                s_host_rx_buf[4])))
            {
                int16_t v_lin = (int16_t)((uint16_t)(s_host_rx_buf[1] << 8) |
                                                          s_host_rx_buf[2]);
                int16_t v_ang = (int16_t)((uint16_t)(s_host_rx_buf[3] << 8) |
                                                          s_host_rx_buf[4]);

                s_host_v_mps   = (float)v_lin / 1000.0f;    /* mm/s -> m/s */
                s_host_w_radps = (float)v_ang / 1000.0f;    /* mrad/s -> rad/s */
                s_host_cmd_pending = 1u;
            }
        }
        break;

    default:
        s_host_rx_state = 0u;
        break;
    }
}

/* ======================== 公有函数 ======================== */

void UART_Host_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    uint16_t i;

    if ((Buffer == 0) || (Length == 0u)) {
        return;
    }

    for (i = 0u; i < Length; i++) {
        Host_Input_Byte(Buffer[i]);
    }
}

void Host_ApplyPendingCommand(void)
{
    if (s_host_cmd_pending != 0u) {
        /* 收到新指令: 应用它, 并复位超时计数 */
        s_host_cmd_pending = 0u;
        s_host_ever_got_cmd = 1u;
        s_host_idle_ticks  = 0u;
        s_host_stopped     = 0u;
        Chassis_SetTwist(s_host_v_mps, s_host_w_radps);
        return;
    }

    /* 没收到新指令 */
    if (s_host_ever_got_cmd == 0u) {
        return;   /* 还没收到过任何指令, 不处理超时 */
    }

    s_host_idle_ticks++;
    if ((s_host_idle_ticks >= HOST_CMD_TIMEOUT_TICKS) && (s_host_stopped == 0u)) {
        s_host_stopped = 1u;
        Chassis_Stop();   /* 超时停车 */
    }
}

void Host_SendOdom(void)
{
    const Chassis_Odom_t *od = Chassis_GetOdom();
    uint32_t sample_tick;
    int32_t left_count;
    int32_t right_count;
    float x_mm;
    float y_mm;
    float theta_rad;
    uint8_t cksum;
    uint8_t i;

    if (od == 0) {
        return;
    }

    x_mm      = od->x_m * 1000.0f;      /* m -> mm */
    y_mm      = od->y_m * 1000.0f;      /* m -> mm */
    theta_rad = od->theta_rad;          /* 本来就是弧度 */

    sample_tick = nowtime;
    left_count = Encoder_GetCount(ENC_LEFT);
    right_count = Encoder_GetCount(ENC_RIGHT);

    s_odom_buf[0] = HOST_HEAD_ODOM_V2;
    s_odom_buf[1] = 1u;                         /* 协议版本 */
    memcpy(&s_odom_buf[2],  &s_odom_sequence, 2);
    memcpy(&s_odom_buf[4],  &sample_tick,     4);
    memcpy(&s_odom_buf[8],  &x_mm,            4);
    memcpy(&s_odom_buf[12], &y_mm,            4);
    memcpy(&s_odom_buf[16], &theta_rad,       4);
    memcpy(&s_odom_buf[20], &left_count,      4);
    memcpy(&s_odom_buf[24], &right_count,     4);

    cksum = 0u;
    for (i = 1u; i <= 27u; i++) {
        cksum += s_odom_buf[i];
    }
    s_odom_buf[28] = cksum;
    s_odom_buf[29] = HOST_FOOTER;

    UART_Send_Data(&huart2, s_odom_buf, 30u);
    s_odom_sequence++;
}
