/**
 * @file    Servo.h
 * @brief   SG90 舵机驱动模块 — 基于 HAL 库的四路舵机控制
 * @note
 *          硬件平台: STM32F407VGT6
 *          定时器:   TIM4, 4路PWM输出 (CH1~CH4)
 *          GPIO:     PD12(SG901), PD13(SG902), PD14(SG903), PD15(SG904)
 *
 *          TIM4 时钟配置:
 *            - APB1 定时器时钟 = 84MHz (2 × APB1 = 2 × 42MHz)
 *            - 预分频器(PSC) = 84-1  →  计数器时钟 = 84MHz / 84 = 1MHz
 *            - 自动重载值(ARR) = 20000-1  →  PWM周期 = 20000 / 1MHz = 20ms (50Hz)
 *            - 1 个计数器 tick = 1μs
 *
 *          SG90 舵机控制原理:
 *            - 控制信号: PWM, 周期 20ms (50Hz) — 标准舵机刷新率
 *            - 脉冲宽度范围: 0.5ms ~ 2.5ms
 *            - 0.5ms 脉冲 → 0° (舵机归零)
 *            - 1.5ms 脉冲 → 90° (居中)
 *            - 2.5ms 脉冲 → 180° (最大角度)
 *            - 换算公式: pulse = 500 + angle * 2000 / 180
 *                          (500 = 0.5ms@1MHz, 2500 = 2.5ms@1MHz)
 * @author  小雨
 * @date    2026-07-27
 */

#ifndef SERVO_H
#define SERVO_H

#include "bsp.h"

/* ================================================================
 *                          舵机参数宏定义
 * ================================================================ */

/**
 * @brief 舵机PWM脉冲宽度 (单位: timer tick,  @ 1MHz 计数器时钟)
 * @note  1 tick = 1μs
 *        SERVO_MIN_PULSE =  500 tick =  500μs = 0.5ms (对应0°)
 *        SERVO_MID_PULSE = 1500 tick = 1500μs = 1.5ms (对应90°)
 *        SERVO_MAX_PULSE = 2500 tick = 2500μs = 2.5ms (对应180°)
 */
#define SERVO_MIN_PULSE     ((uint16_t)500)    /**< 0° 对应脉冲 (0.5ms) */
#define SERVO_MAX_PULSE     ((uint16_t)2500)   /**< 180° 对应脉冲 (2.5ms) */
#define SERVO_MID_PULSE     ((uint16_t)1500)   /**< 90° 对应脉冲 (1.5ms) */
#define SERVO_PULSE_RANGE   ((uint16_t)2000)   /**< 脉冲变化范围 (2500-500) */

/** @brief PWM周期 (timer tick 数, ARR 值) */
#define SERVO_PWM_PERIOD    ((uint16_t)20000)

/** @brief 角度限幅 */
#define SERVO_ANGLE_MIN     ((uint8_t)0)
#define SERVO_ANGLE_MAX     ((uint8_t)180)

/** @brief 平滑运动参数: 每步间隔 (ms), 与舵机 PWM 周期一致 */
#define SERVO_SMOOTH_STEP_MS ((uint16_t)20)

/* ================================================================
 *                          舵机通道枚举
 * ================================================================ */

/**
 * @brief 舵机通道编号
 * @note  通道与硬件引脚的对应关系:
 *          SERVO_CH_1 → SG901 → PD12 → TIM4_CH1
 *          SERVO_CH_2 → SG902 → PD13 → TIM4_CH2
 *          SERVO_CH_3 → SG903 → PD14 → TIM4_CH3
 *          SERVO_CH_4 → SG904 → PD15 → TIM4_CH4
 */
typedef enum {
    SERVO_CH_1 = 1,     /**< 舵机1: SG901, TIM4_CH1 */
    SERVO_CH_2 = 2,     /**< 舵机2: SG902, TIM4_CH2 */
    SERVO_CH_3 = 3,     /**< 舵机3: SG903, TIM4_CH3 */
    SERVO_CH_4 = 4      /**< 舵机4: SG904, TIM4_CH4 */
} Servo_Channel_t;

/* ================================================================
 *                         API 函数声明
 * ================================================================ */

/**
 * @brief  舵机模块初始化
 * @note   启动 TIM4 全部四路 PWM 输出，所有舵机归中 (90°)
 *         应在 MX_TIM4_Init() 之后调用
 * @retval 无
 */
void Servo_Init(void);

/**
 * @brief  舵机模块反初始化
 * @note   停止 TIM4 全部四路 PWM 输出，释放定时器资源
 * @retval 无
 */
void Servo_DeInit(void);

/**
 * @brief  启动单个舵机通道的 PWM 输出
 * @param  ch  舵机通道编号 @ref Servo_Channel_t (SERVO_CH_1 ~ SERVO_CH_4)
 * @retval 无
 */
void Servo_Start(uint8_t ch);

/**
 * @brief  停止单个舵机通道的 PWM 输出
 * @param  ch  舵机通道编号 @ref Servo_Channel_t (SERVO_CH_1 ~ SERVO_CH_4)
 * @retval 无
 */
void Servo_Stop(uint8_t ch);

/**
 * @brief  设置舵机角度 (立即跳转)
 * @param  ch    舵机通道编号 (SERVO_CH_1 ~ SERVO_CH_4)
 * @param  angle 目标角度 (0° ~ 180°)，超出范围自动限幅
 * @note   换算公式: CCR = 500 + angle * 2000 / 180
 *         其中 500 对应 0.5ms (0°), 2500 对应 2.5ms (180°)
 * @retval 无
 */
void Servo_SetAngle(uint8_t ch, uint8_t angle);

/**
 * @brief  设置舵机原始脉冲宽度 (CCR 值)
 * @param  ch    舵机通道编号 (SERVO_CH_1 ~ SERVO_CH_4)
 * @param  pulse 脉冲宽度 (timer tick 数, 范围 1000~5000, 超出自动限幅)
 * @note   高级接口, 用于需要精确脉冲控制的场景
 *         通常建议使用 Servo_SetAngle()
 * @retval 无
 */
void Servo_SetPulse(uint8_t ch, uint16_t pulse);

/**
 * @brief  获取舵机当前角度
 * @param  ch  舵机通道编号 (SERVO_CH_1 ~ SERVO_CH_4)
 * @retval 当前角度 (0° ~ 180°)
 */
uint8_t Servo_GetAngle(uint8_t ch);

/**
 * @brief  舵机平滑运动 (线性插值)
 * @param  ch           舵机通道编号 (SERVO_CH_1 ~ SERVO_CH_4)
 * @param  target_angle 目标角度 (0° ~ 180°)
 * @param  duration_ms  运动总时长 (单位: 毫秒)
 * @note   内部按 SERVO_SMOOTH_STEP_MS (20ms) 为步进间隔,
 *         将总时长等分为 steps = duration_ms / 20 步,
 *         每步延时 20ms, 实现平滑过渡
 * @warning 此函数内部使用 delay_ms() 实现阻塞延时,
 *          调用期间会阻塞当前任务, 慎用于实时性要求高的场景
 * @retval 无
 */
void Servo_SmoothMove(uint8_t ch, uint8_t target_angle, uint16_t duration_ms);

#endif /* SERVO_H */
