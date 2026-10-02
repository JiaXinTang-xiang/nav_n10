/**
 * @file    Chassis_protocol.h
 * @brief   底盘串口协议 (F407 <-> Jetson Nano), 与 wheeltec_chassis 桥接一致
 *
 * 协议 (与 ros2_ws/src/wheeltec_chassis/chassis_bridge.py 完全对应):
 *
 *   上位机 -> F407 (速度指令):
 *     [0xBB][v_linear int16 大端, mm/s][v_angular int16 大端, mrad/s][校验][0x55]
 *     共 7 字节, 校验 = (v_hi + v_lo + w_hi + w_lo) & 0xFF
 *
 *   F407 -> 上位机 (里程计):
 *     [0xCC][x float 小端, mm][y float 小端, mm][theta float 小端, rad][校验][0x55]
 *     共 15 字节, 校验 = (x+y+theta 共 12 字节之和) & 0xFF
 *
 * 职责:
 *   - 解析 0xBB 帧 -> v_linear/v_angular -> Chassis_SetTwist()
 *   - 周期发送 0xCC 帧 (x/y/theta 来自 Chassis_GetOdom())
 */

#ifndef CHASSIS_PROTOCOL_H
#define CHASSIS_PROTOCOL_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 协议常量 ======================== */
#define HOST_HEAD_CMD    0xBBu    /* 上位机 -> F407 速度指令帧头 */
#define HOST_HEAD_ODOM   0xCCu    /* F407 -> 上位机 里程计帧头 */
#define HOST_FOOTER      0x55u    /* 帧尾 */

/* ======================== 接口 ======================== */

/**
 * @brief  USART2 接收回调 (DMA 空闲中断里调用)
 * @note   把字节流喂给 0xBB 帧解析状态机。上位机发一帧 7 字节,
 *         解析成功后缓存 v/w, 由 Chassis_Update() 里统一应用。
 */
void UART_Host_Call_Back(uint8_t *Buffer, uint16_t Length);

/**
 * @brief  应用上位机最新速度指令 (放在 Chassis_Update 200Hz 里调用)
 * @note   若解析到了新指令, 调 Chassis_SetTwist 把 v/w 转成左右轮速
 */
void Host_ApplyPendingCommand(void);

/**
 * @brief  发送里程计 0xCC 帧 (放在 50Hz 定时任务里调用)
 */
void Host_SendOdom(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_PROTOCOL_H */
