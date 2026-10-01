/**
 * @file    Relay.c
 * @brief   四路继电器驱动模块实现
 * @note    高电平吸合 / 低电平断开
 *
 *          【移植指南 — 从 STM32 到 TI (TM4C / MSP430 等)】
 *          只需改两处:
 *            1. 修改 relay_pins[] 表中 port/pin 定义, 匹配 TI 的 GPIO
 *            2. 修改 relay_write() 函数内的 GPIO 写操作, 换成 TI 的 API
 *          其余代码 (API、状态管理、通道校验) 无需改动
 *
 * @author  小雨
 * @date    2026-07-27
 */

#include "Relay.h"

/* ================================================================
 *              移植适配层 — 换平台只改下面这些宏
 * ================================================================
 *  移植到 TI 时:
 *    1. 把下面 4 组宏改成 TI 的 GPIO 基址和引脚编号
 *    2. 把 relay_write() 里的 HAL_GPIO_WritePin() 换成 TI 的 API
 *    其余代码无需改动
 */

/** @brief 继电器1 — IO1 — PE5 */
#define RELAY1_PORT    GPIOE
#define RELAY1_PIN     IO1_Pin

/** @brief 继电器2 — IO2 — PE6 */
#define RELAY2_PORT    GPIOE
#define RELAY2_PIN     IO2_Pin

/** @brief 继电器3 — IO3 — PB7 */
#define RELAY3_PORT    GPIOB
#define RELAY3_PIN     IO3_Pin

/** @brief 继电器4 — IO4 — PA4 */
#define RELAY4_PORT    GPIOA
#define RELAY4_PIN     IO4_Pin

/**
 * @brief  底层 GPIO 写操作 (平台相关)
 * @param  idx   通道索引 (0~3)
 * @param  level true=高电平(ON), false=低电平(OFF)
 * @note   移植到 TI 时替换 HAL_GPIO_WritePin 为对应 API
 */
static void relay_write(uint8_t idx, bool level)
{
    GPIO_PinState state = level ? GPIO_PIN_SET : GPIO_PIN_RESET;

    switch (idx) {
        case 0:  HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, state); break;
        case 1:  HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, state); break;
        case 2:  HAL_GPIO_WritePin(RELAY3_PORT, RELAY3_PIN, state); break;
        case 3:  HAL_GPIO_WritePin(RELAY4_PORT, RELAY4_PIN, state); break;
        default: break;
    }
}

/* ================================================================
 *              移植适配层结束 — 以下代码平台无关
 * ================================================================ */

/** @brief 四路继电器当前状态 (true=ON, false=OFF) */
static bool relay_state[4] = {false, false, false, false};

/**
 * @brief  验证通道编号是否合法
 * @param  ch  通道编号
 * @retval true  合法
 * @retval false 非法
 */
static bool Relay_IsValidChannel(uint8_t ch)
{
    return (ch >= RELAY_CH_1 && ch <= RELAY_CH_4);
}

/* ================================================================
 *                        API 函数实现
 * ================================================================ */

/**
 * @brief  继电器模块初始化
 * @note   确保全部处于断开状态 (低电平)
 *         GPIO 时钟和模式已由 CubeMX 的 MX_GPIO_Init() 配置好
 */
void Relay_Init(void)
{
    Relay_OFF_All();
}

/**
 * @brief  继电器吸合 (高电平)
 */
void Relay_ON(uint8_t ch)
{
    if (!Relay_IsValidChannel(ch)) return;

    uint8_t idx = ch - 1;
    relay_state[idx] = true;
    relay_write(idx, true);
}

/**
 * @brief  继电器断开 (低电平)
 */
void Relay_OFF(uint8_t ch)
{
    if (!Relay_IsValidChannel(ch)) return;

    uint8_t idx = ch - 1;
    relay_state[idx] = false;
    relay_write(idx, false);
}

/**
 * @brief  继电器状态翻转
 */
void Relay_Toggle(uint8_t ch)
{
    if (!Relay_IsValidChannel(ch)) return;

    uint8_t idx = ch - 1;
    relay_state[idx] = !relay_state[idx];
    relay_write(idx, relay_state[idx]);
}

/**
 * @brief  设置继电器到指定状态
 */
void Relay_Set(uint8_t ch, bool state)
{
    if (state)
        Relay_ON(ch);
    else
        Relay_OFF(ch);
}

/**
 * @brief  获取继电器当前状态
 */
bool Relay_GetState(uint8_t ch)
{
    if (!Relay_IsValidChannel(ch)) return false;
    return relay_state[ch - 1];
}

/**
 * @brief  全部吸合
 */
void Relay_ON_All(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        relay_state[i] = true;
        relay_write(i, true);
    }
}

/**
 * @brief  全部断开
 */
void Relay_OFF_All(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        relay_state[i] = false;
        relay_write(i, false);
    }
}

/* ================================================================
 *                     main.c 调用示例 (供参考)
 * ================================================================
 *
 *   // -------------------- 初始化 --------------------
 *   // 在 main() 中 MX_GPIO_Init() 之后调用一次
 *   Relay_Init();       // 全部断开
 *
 *   // -------------------- 基本开关 --------------------
 *   Relay_ON(RELAY_CH_1);              // IO1 吸合
 *   Relay_OFF(RELAY_CH_1);             // IO1 断开
 *   Relay_Set(RELAY_CH_2, true);       // IO2 吸合 (等效 Relay_ON)
 *   Relay_Set(RELAY_CH_2, false);      // IO2 断开 (等效 Relay_OFF)
 *
 *   // -------------------- 翻转 --------------------
 *   Relay_Toggle(RELAY_CH_3);          // IO3: ON→OFF 或 OFF→ON
 *
 *   // -------------------- 查询状态 --------------------
 *   if (Relay_GetState(RELAY_CH_1)) {
 *       // IO1 当前是吸合状态
 *   }
 *
 *   // -------------------- 批量操作 --------------------
 *   Relay_ON_All();                    // 四路全部吸合
 *   delay_ms(500);
 *   Relay_OFF_All();                   // 四路全部断开
 *
 *   // -------------------- 顺序控制示例 --------------------
 *   Relay_ON(RELAY_CH_1);   delay_ms(100);
 *   Relay_ON(RELAY_CH_2);   delay_ms(100);
 *   Relay_ON(RELAY_CH_3);   delay_ms(100);
 *   Relay_ON(RELAY_CH_4);              // 四路依次吸合
 *
 *   ============================================================
 *   移植到 TI 的步骤:
 *   1. 修改 RELAY1~4_PORT / RELAY1~4_PIN 宏, 匹配 TI 硬件
 *   2. 修改 relay_write() 里的 case 分支, 换成 TI SDK 的 GPIO API
 *   3. 其余代码无需改动
 *   ============================================================
 */
