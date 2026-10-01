#include "stm32f10x.h"
#include <string.h>
#include "Serial.h"

/* ========== 接收缓冲区与标志 ========== */
uint8_t Serial_RxPacket[16];          // 最大15字节 (里程计回传可能不做接收, 但留空间)
volatile uint8_t Serial_RxFlag = 0;   // 视觉数据包接收完成 (0xAA)
volatile uint8_t cmd_vel_flag  = 0;   // 速度指令接收完成 (0xBB)

/* ========== 解析后的数据 ========== */
uint16_t Serial_X, Serial_Y, Serial_Area;
float center_x = 0, center_y = 0, area_value = 0;
float cmd_v_linear  = 0;
float cmd_v_angular = 0;

/**
 * @brief  串口初始化 (USART1, 115200, PA9/PA10)
 */
void Serial_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

/* ========== 发送函数 ========== */

void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    for (uint16_t i = 0; i < Length; i++)
        Serial_SendByte(Array[i]);
}

/* ========== 视觉协议 (0xAA) ========== */

uint8_t Serial_GetRxFlag(void)
{
    if (Serial_RxFlag)
    {
        Serial_RxFlag = 0;
        return 1;
    }
    return 0;
}

void Serial_GetData(uint16_t *X, uint16_t *Y, uint16_t *Area)
{
    *X = (Serial_RxPacket[2] << 8) | Serial_RxPacket[3];
    *Y = (Serial_RxPacket[4] << 8) | Serial_RxPacket[5];
    *Area = (Serial_RxPacket[6] << 8) | Serial_RxPacket[7];

    Serial_X = *X;
    Serial_Y = *Y;
    Serial_Area = *Area;
}

/* ========== 速度指令协议 (0xBB) ========== */

uint8_t Serial_GetCmdVelFlag(void)
{
    if (cmd_vel_flag)
    {
        cmd_vel_flag = 0;
        return 1;
    }
    return 0;
}

/**
 * @brief  发送里程计数据到Jetson
 *         协议: 0xCC + x(4B float) + y(4B float) + theta(4B float) + cksum + 0x55
 * @param  x, y: 坐标 (mm)
 * @param  theta: 朝向角 (rad)
 */
void Serial_SendOdometry(float x, float y, float theta)
{
    uint8_t buf[15];
    uint8_t *p;

    buf[0] = 0xCC;

    /* x (4 bytes, float) */
    p = (uint8_t *)&x;
    buf[1] = p[0]; buf[2] = p[1]; buf[3] = p[2]; buf[4] = p[3];

    /* y (4 bytes, float) */
    p = (uint8_t *)&y;
    buf[5] = p[0]; buf[6] = p[1]; buf[7] = p[2]; buf[8] = p[3];

    /* theta (4 bytes, float) */
    p = (uint8_t *)&theta;
    buf[9]  = p[0]; buf[10] = p[1]; buf[11] = p[2]; buf[12] = p[3];

    /* checksum (bytes 1-12的和) */
    uint8_t cksum = 0;
    for (int i = 1; i <= 12; i++)
        cksum += buf[i];
    buf[13] = cksum;
    buf[14] = 0x55;

    Serial_SendArray(buf, 15);
}

/* ========== 串口中断 — 双协议状态机 ========== */

void USART1_IRQHandler(void)
{
    static uint8_t RxState    = 0;    // 0=等待帧头, 1=接收数据
    static uint8_t RxIndex    = 0;    // 当前写入位置
    static uint8_t RxProtocol = 0;    // 0=未知, 1=0xAA视觉, 2=0xBB速度
    static uint8_t RxLen      = 0;    // 期望总长度

    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        uint8_t RxData = USART_ReceiveData(USART1);

        if (RxState == 0)  // 等待帧头
        {
            if (RxData == 0xAA)          // 视觉协议: 11字节
            {
                RxProtocol = 1;
                RxLen      = 11;
                Serial_RxPacket[0] = RxData;
                RxIndex = 1;
                RxState = 1;
            }
            else if (RxData == 0xBB)     // 速度指令: 7字节
            {
                RxProtocol = 2;
                RxLen      = 7;
                Serial_RxPacket[0] = RxData;
                RxIndex = 1;
                RxState = 1;
            }
            // 其他字节忽略
        }
        else if (RxState == 1)  // 接收数据
        {
            if (RxIndex < RxLen)
            {
                Serial_RxPacket[RxIndex] = RxData;
                RxIndex++;
            }

            if (RxIndex == RxLen)  // 收满
            {
                /* 校验帧尾 */
                if (Serial_RxPacket[RxLen - 1] == 0x55)
                {
                    if (RxProtocol == 1)  // 视觉数据
                    {
                        Serial_RxFlag = 1;
                    }
                    else if (RxProtocol == 2)  // 速度指令
                    {
                        /* 校验checksum: bytes 1~4的和 */
                        uint8_t cksum = Serial_RxPacket[1] + Serial_RxPacket[2]
                                      + Serial_RxPacket[3] + Serial_RxPacket[4];
                        if (cksum == Serial_RxPacket[5])
                        {
                            /* 解析: 大端序 int16 */
                            int16_t vl = (int16_t)((Serial_RxPacket[1] << 8) | Serial_RxPacket[2]);
                            int16_t va = (int16_t)((Serial_RxPacket[3] << 8) | Serial_RxPacket[4]);

                            cmd_v_linear  = (float)vl;   // mm/s
                            cmd_v_angular = (float)va;   // mrad/s
                            cmd_vel_flag  = 1;
                        }
                    }
                }
                /* 重置状态机 */
                RxState    = 0;
                RxIndex    = 0;
                RxProtocol = 0;
            }
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
