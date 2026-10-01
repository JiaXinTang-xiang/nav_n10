/**
 * @file    Relay.h
 * @brief   四路继电器驱动模块 — 高电平开 / 低电平关
 * @note
 *          硬件平台: STM32F407VGT6 (HAL 库)
 *          移植说明: 本模块 GPIO 操作集中在 Relay.c 的映射表和一个
 *                   写引脚函数中，移植到 TI 只需改那两处即可。
 *
 *          引脚映射:
 *            RELAY_CH_1 → IO1 → PE5
 *            RELAY_CH_2 → IO2 → PE6
 *            RELAY_CH_3 → IO3 → PB7
 *            RELAY_CH_4 → IO4 → PA4
 *
 *          电平逻辑:
 *            高电平 (SET)   → 继电器吸合 (ON)
 *            低电平 (RESET) → 继电器断开 (OFF)
 *
 * @author  小雨
 * @date    2026-07-27
 */

#ifndef RELAY_H
#define RELAY_H

#include "bsp.h"

/* ================================================================
 *                          继电器通道枚举
 * ================================================================ */

/**
 * @brief 继电器通道编号
 * @note  通道与硬件引脚的对应关系:
 *          RELAY_CH_1 → IO1 → PE5
 *          RELAY_CH_2 → IO2 → PE6
 *          RELAY_CH_3 → IO3 → PB7
 *          RELAY_CH_4 → IO4 → PA4
 */
typedef enum {
    RELAY_CH_1 = 1,     /**< 继电器1: IO1, PE5 */
    RELAY_CH_2 = 2,     /**< 继电器2: IO2, PE6 */
    RELAY_CH_3 = 3,     /**< 继电器3: IO3, PB7 */
    RELAY_CH_4 = 4      /**< 继电器4: IO4, PA4 */
} Relay_Channel_t;

/* ================================================================
 *                         API 函数声明
 * ================================================================ */

/**
 * @brief  继电器模块初始化
 * @note   将全部四路继电器初始化为断开状态 (低电平)
 *         引脚时钟和模式已由 CubeMX MX_GPIO_Init() 配置,
 *         此处仅确保输出为低电平
 * @retval 无
 */
void Relay_Init(void);

/**
 * @brief  继电器吸合 (ON)
 * @param  ch  通道编号 @ref Relay_Channel_t (RELAY_CH_1 ~ RELAY_CH_4)
 * @note   输出高电平 → 继电器吸合
 * @retval 无
 */
void Relay_ON(uint8_t ch);

/**
 * @brief  继电器断开 (OFF)
 * @param  ch  通道编号 @ref Relay_Channel_t (RELAY_CH_1 ~ RELAY_CH_4)
 * @note   输出低电平 → 继电器断开
 * @retval 无
 */
void Relay_OFF(uint8_t ch);

/**
 * @brief  继电器状态翻转
 * @param  ch  通道编号 @ref Relay_Channel_t (RELAY_CH_1 ~ RELAY_CH_4)
 * @note   ON → OFF, OFF → ON
 * @retval 无
 */
void Relay_Toggle(uint8_t ch);

/**
 * @brief  设置继电器状态
 * @param  ch    通道编号 @ref Relay_Channel_t (RELAY_CH_1 ~ RELAY_CH_4)
 * @param  state true=ON(高电平), false=OFF(低电平)
 * @retval 无
 */
void Relay_Set(uint8_t ch, bool state);

/**
 * @brief  获取继电器当前状态
 * @param  ch  通道编号 @ref Relay_Channel_t (RELAY_CH_1 ~ RELAY_CH_4)
 * @retval true  = 吸合 (ON)
 * @retval false = 断开 (OFF)
 */
bool Relay_GetState(uint8_t ch);

/**
 * @brief  全部继电器吸合
 * @retval 无
 */
void Relay_ON_All(void);

/**
 * @brief  全部继电器断开
 * @retval 无
 */
void Relay_OFF_All(void);

#endif /* RELAY_H */
