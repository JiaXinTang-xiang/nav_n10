#ifndef BSP_LED_H
#define BSP_LED_H
#include "bsp.h"

/* ===================== LED1 / LED2 控制宏 (低电平有效) ===================== */

#define LED1(x)       HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (x) ? GPIO_PIN_RESET : GPIO_PIN_SET)
#define LED2(x)       HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (x) ? GPIO_PIN_RESET : GPIO_PIN_SET)
#define LED1_toggle() HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)
#define LED2_toggle() HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)

/* ===================== RGB LED 控制宏 (WS2812 单总线) ===================== */

#define RGB(x)        HAL_GPIO_WritePin(RGB_GPIO_Port, RGB_Pin, (x) ? GPIO_PIN_RESET : GPIO_PIN_SET)

/* ===================== RGB LED 函数 ===================== */

/**
 * @brief  WS2812 协议发送 24-bit RGB 数据
 * @param  R  红色分量 (0-255)
 * @param  G  绿色分量 (0-255)
 * @param  B  蓝色分量 (0-255)
 */
void RGB_Control(uint8_t R, uint8_t G, uint8_t B);


typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} LED_Struct;

void led_on(LED_Struct *led);
void led_off(LED_Struct *led);
void led_toggle(LED_Struct *led);


#endif // BSP_LED_H

/* ===================== 通用 LED 结构体 ===================== */



