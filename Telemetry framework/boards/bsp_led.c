#include "bsp_led.h"

/* ===================== WS2812 快速 IO ===================== */
#define WS_HIGH()  RGB_GPIO_Port->BSRR = RGB_Pin
#define WS_LOW()   RGB_GPIO_Port->BSRR = (uint32_t)RGB_Pin << 16

/* ===================== NOP 延时 ===================== */
/*
 * 168MHz 下每轮 do{NOP}while(cnt--) 约 3~4 周期 ≈ 20ns
 *
 * WS2812 要求 (宽松容差):
 *   0-bit:  HIGH 短 (200-500ns),  LOW 长 (650-1000ns)
 *   1-bit:  HIGH 长 (550-950ns),  LOW 短 (300-650ns)
 *   RESET:  LOW > 50us
 *
 * 下面参数: BIT0 高短低长, BIT1 高长低短, 差值足够 WS2812 分辨
 */
#define WS2812_BIT0_HI   5    /* ~100ns  */
#define WS2812_BIT0_LO   50   /* ~1000ns */
#define WS2812_BIT1_HI   45   /* ~900ns  */
#define WS2812_BIT1_LO   8    /* ~160ns  */

__attribute__((noinline))
static void ws2812_nop_delay(uint32_t cnt)
{
    do {
        __NOP();
    } while (cnt--);
}

__attribute__((noinline))
static void ws2812_bit1(void)
{
    WS_HIGH();
    ws2812_nop_delay(WS2812_BIT1_HI);
    WS_LOW();
    ws2812_nop_delay(WS2812_BIT1_LO);
}

__attribute__((noinline))
static void ws2812_bit0(void)
{
    WS_HIGH();
    ws2812_nop_delay(WS2812_BIT0_HI);
    WS_LOW();
    ws2812_nop_delay(WS2812_BIT0_LO);
}

/* ===================== 发送函数 ===================== */

void RGB_Control(uint8_t R, uint8_t G, uint8_t B)
{
    /* GRB 顺序: G[23:16] R[15:8] B[7:0] */
    uint32_t grb = ((uint32_t)G << 16) | ((uint32_t)R << 8) | B;

    /* RESET: 拉低 2ms (NOP忙等, ISR/main通用) */
    WS_LOW();
    ws2812_nop_delay(100000);  /* ~2ms @168MHz, 替代 HAL_Delay */

    for (int8_t i = 23; i >= 0; i--)
    {
        if (grb & ((uint32_t)1 << i))
            ws2812_bit1();
        else
            ws2812_bit0();
    }

    /* 拉高锁存 */
    WS_HIGH();
}

/* ===================== 通用 LED 操作 ===================== */

/**
 * @brief  打开 LED (低电平有效)
 * @param  led  指向 LED_Struct 的指针
 */
void led_on(LED_Struct *led)
{
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}

/**
 * @brief  关闭 LED (低电平有效)
 * @param  led  指向 LED_Struct 的指针
 */
void led_off(LED_Struct *led)
{
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
}

/**
 * @brief  翻转 LED 状态
 * @param  led  指向 LED_Struct 的指针
 */
void led_toggle(LED_Struct *led)
{
    HAL_GPIO_TogglePin(led->port, led->pin);
}

