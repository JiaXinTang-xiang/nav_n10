/**
 * @file    encoder.h
 * @brief   两轮 AB 相霍尔编码器解码 (不改线方案的软件正交解码)
 * @note    硬件连接 (已固定在 PCB 上, 不可改线):
 *          - 右轮: A -> PA2 (TIM2_CH3)   B -> PA3 (普通输入)
 *          - 左轮: A -> PE13 (TIM1_CH3)  B -> PE11 (普通输入)
 *
 *          为什么不能用硬件 Encoder Mode:
 *            STM32 的正交编码器模式 (SMS=001/010/011) 只能把 TI1/TI2 映射到
 *            CH1/CH2, CH3/CH4 无法作为编码器输入, 因此 CubeMX 在 Encoder Mode
 *            下不会接受这 4 个引脚。本驱动改用等效方案:
 *
 *            - CH3 (PE13 / PA2) 配成"双边沿输入捕获", 每来一个边沿进一次中断,
 *              相当于对 A 相做 2 倍频计数;
 *            - CH2 (PE11 / PA3) 不配定时器, 当普通数字输入用, 在中断里读它的
 *              电平: A/B 异电平 => 原始正转, 同电平 => 原始反转。
 *
 *            结果: 每转脉冲数 = 线数 x 4 x 减速比 / 2 = 线数 x 减速比 x 2
 *                  13 线 / 1:30 => 780 count/圈, 0.262 mm/count
 *
 *          这种接法下 CH3/CH4 是独立捕获通道 (只有 CH1/CH2 共用 CCMR1),
 *          所以 CH2 可以老老实实当输入用, 互不干扰。
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

/* ======================== C/C++ 链接说明 ======================== */
/* 本工程在 Keil AC5 下所有 .c 都按 C++ 编译(实测 encoder.o 里 Load/TB6612_Init
   等符号都是 C++ 修饰名), 所以头文件必须显式声明 extern "C", 否则:
     - encoder.c 定义出来的是 C++ 修饰名 _Z18Encoder_CaptureIRQP11TIM_TypeDef
     - 而调用方要的是 C 名字 Encoder_CaptureIRQ
   => L6218E: Undefined symbol Encoder_CaptureIRQ                     */
#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 机械与编码器参数 ======================== */
/* 标定时只需要改这几个数 */

#define ENC_LINES_PER_REV    13.0f      /* 编码器线数 (电机轴每转 A 相脉冲数) */
#define ENC_GEAR_RATIO       30.0f      /* 减速比 */
#define ENC_DECODE_FACTOR    2.0f       /* 本方案为双边沿(2 倍频); 若改为四倍频软件解码改 4.0f */

#define WHEEL_DIAMETER_M     0.065f     /* 轮径 65 mm */
#define WHEEL_RADIUS_M       0.0325f    /* 轮半径 */
/* 几何中心轮距为 161 mm（188 mm 外侧总宽 - 27 mm 胎宽）。
   实车原地旋转 360 度时，161 mm 参数只累计约 320 度，因此按
   161 * 320 / 360 = 143.1 mm 修正为有效轮距。后续用顺/逆时针复测微调。 */
#define WHEEL_SEPARATION_M   0.1431f

/* ======================== 派生常量 ======================== */
#define WHEEL_CIRCUMFERENCE_M  (3.14159265358979f * WHEEL_DIAMETER_M)
#define ENC_COUNTS_PER_REV     (ENC_LINES_PER_REV * ENC_GEAR_RATIO * ENC_DECODE_FACTOR)

/* 2026-10-06 实测，每侧向前手转 10 圈并重复 3 次：
   左: 7830, 7799, 7810 -> 781.3 count/圈
   右: 7829, 7830, 7801 -> 782.0 count/圈
   左右只差约 0.09%，说明捕获/解码一致；仍保留左右独立标定。 */
#define ENC_LEFT_COUNTS_PER_WHEEL_REV   781.3f
#define ENC_RIGHT_COUNTS_PER_WHEEL_REV  782.0f

#define ENC_LEFT_METERS_PER_COUNT  (WHEEL_CIRCUMFERENCE_M / ENC_LEFT_COUNTS_PER_WHEEL_REV)
#define ENC_RIGHT_METERS_PER_COUNT (WHEEL_CIRCUMFERENCE_M / ENC_RIGHT_COUNTS_PER_WHEEL_REV)

/* ======================== 编码器编号 ======================== */
typedef enum {
    ENC_LEFT  = 0,      /* TIM1, PE11(B/CH2) + PE13(A/CH3) */
    ENC_RIGHT = 1,      /* TIM2, PA3(B/CH4)  + PA2(A/CH3)  */
    ENC_COUNT
} EncoderID_t;

/* ======================== 编码器状态 ======================== */
typedef struct {
    int32_t  count;         /* 累计计数 (自 Encoder_Reset 起, 带符号) */
    int16_t  delta;         /* 上一次 Encoder_Update 以来的增量 */
    float    distance_m;    /* 累计行驶距离 (米, 带符号) */
    float    velocity_mps;  /* 轮子线速度 (米/秒, 带符号) */
    int8_t   dir_sign;      /* 方向符号: +1 或 -1, 用来纠正接线极性
                               注意必须是 int8_t! uint8_t 存 -1 会变成 255,
                               增量会被乘以 255 而不是取反(踩过这个坑) */
    uint32_t io_error;      /* 计数溢出/异常统计 (预留) */
} Encoder_t;

/* ======================== 接口 ======================== */

/**
 * @brief  初始化两路编码器 (必须在 MX_TIM1_Init / MX_TIM2_Init 之后调用)
 * @note   会开启 TIM1_CC 和 TIM2 中断, 不需要 CubeMX 里配 NVIC
 */
void Encoder_Init(void);

/**
 * @brief  输入捕获中断服务, 由 TIM1_CC_IRQHandler / TIM2_IRQHandler 调用
 * @param  inst  TIM1 (左轮) 或 TIM2 (右轮)
 * @note   不从 HAL_TIM_IC_CaptureCallback 走, 省掉 HAL 分支判断降低中断延迟
 */
void Encoder_CaptureIRQ(TIM_TypeDef *inst);

/**
 * @brief  周期性读取增量 (建议 5~10ms 调一次, 放在 Task_200Hz)
 * @note   内部用"前后差值"处理 16 位计数器回绕, 不会丢数
 */
void Encoder_Update(void);

/**
 * @brief  清零累计计数与距离 (用于重新标定)
 */
void Encoder_Reset(void);

/**
 * @brief  取编码器状态 (只读)
 * @param  id  ENC_LEFT / ENC_RIGHT
 * @retval 状态指针, 非法 id 返回 NULL
 */
const Encoder_t *Encoder_Get(EncoderID_t id);

/**
 * @brief  取上一次周期的增量计数 (中断安全, 32 位读)
 */
int16_t Encoder_GetDelta(EncoderID_t id);

/**
 * @brief  取累计计数
 */
int32_t Encoder_GetCount(EncoderID_t id);

/**
 * @brief  取走并清零原始中断计数 (仅用于诊断)
 * @note   返回自上次调用以来捕获中断触发的次数。
 *         轮子静止时应该稳定为 0; 持续增长 = 引脚有干扰或编码器输出悬空。
 */
int32_t Encoder_TakeIsrCount(EncoderID_t id);

/**
 * @brief  编码器自检: 软件直接翻转 A 相引脚, 验证整条捕获通路
 * @param  id       要自检的轮子
 * @param  toggles  翻转次数 (建议 200)
 * @retval 自检期间捕获到的 A 相边沿总数
 *
 * @note   不依赖外部信号, 用来区分"代码问题"和"信号问题":
 *           返回 ≈ toggles*2  -> 代码通路正常, 问题在外部信号
 *           返回 0            -> 代码通路有问题
 */
int32_t Encoder_SelfTest(EncoderID_t id, uint16_t toggles);

/**
 * @brief  设置方向符号 (实测"前进时计数为负"时调用, 传 -1 翻转)
 */
void Encoder_SetDirSign(EncoderID_t id, int8_t sign);

float Encoder_GetMetersPerCount(EncoderID_t id);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
