/**
 * @file    encoder.c
 * @brief   两轮 AB 相霍尔编码器软件解码实现
 *
 * 原理:
 *   右轮 PA2 = TIM2_CH3 配"双边沿捕获"(A 相每个边沿进一次中断, 2 倍频)
 *        PA3 = 普通数字输入, 在中断里读电平 (B 相)
 *   左轮 PE13 = TIM1_CH3 同上, PE11 读电平
 *
 *   正交判据 (A 相跳变时刻):
 *       A != B  =>  正转  (+1)
 *       A == B  =>  反转  (-1)
 *   若实测方向相反, 用 Encoder_SetDirSign(id, -1) 翻转即可, 不用改线。
 *
 * 中断预算 (13 线 / 1:30 / 0.065m 轮径 => 7.64 count/mm):
 *   0.5 m/s 时每轮约 3.8 kHz, 两轮 7.6 kHz, ISR 约 30 周期 @84MHz
 *   => CPU 占用 < 0.3%, 余量很大
 */

#include "encoder.h"
#include "tool_delay.h"

/* ======================== 定时器实例与引脚 ======================== */
#define ENC_LEFT_TIM        TIM1
#define ENC_LEFT_IRQn       TIM1_CC_IRQn
#define ENC_LEFT_CH         TIM_CHANNEL_3
#define ENC_LEFT_A_Port     GPIOE
#define ENC_LEFT_A_Pin      GPIO_PIN_13      /* 左轮 A 相 -> TIM1_CH3 */
#define ENC_LEFT_B_Port     GPIOE
#define ENC_LEFT_B_Pin      GPIO_PIN_11      /* 左轮 B 相 -> 普通输入 */

#define ENC_RIGHT_TIM       TIM2
#define ENC_RIGHT_IRQn      TIM2_IRQn
#define ENC_RIGHT_CH        TIM_CHANNEL_3
#define ENC_RIGHT_A_Port    GPIOA
#define ENC_RIGHT_A_Pin     GPIO_PIN_2       /* 右轮 A 相 -> TIM2_CH3 */
#define ENC_RIGHT_B_Port    GPIOA
#define ENC_RIGHT_B_Pin     GPIO_PIN_3       /* 右轮 B 相 -> 普通输入 */

/* 采样周期 (秒), 与 Task_200Hz 对齐 */
#define ENC_SAMPLE_PERIOD_S 0.005f

/* ======================== 私有变量 ======================== */

static TIM_HandleTypeDef htim_enc[ENC_COUNT];

static const TIM_TypeDef   *s_inst[ENC_COUNT]   = { ENC_LEFT_TIM,  ENC_RIGHT_TIM  };
static const uint32_t       s_channel[ENC_COUNT] = { ENC_LEFT_CH,   ENC_RIGHT_CH   };
static GPIO_TypeDef * const s_a_port[ENC_COUNT]  = { ENC_LEFT_A_Port,  ENC_RIGHT_A_Port  };
static const uint16_t       s_a_pin[ENC_COUNT]   = { ENC_LEFT_A_Pin,   ENC_RIGHT_A_Pin   };
static GPIO_TypeDef * const s_b_port[ENC_COUNT]  = { ENC_LEFT_B_Port,  ENC_RIGHT_B_Port  };
static const uint16_t       s_b_pin[ENC_COUNT]   = { ENC_LEFT_B_Pin,   ENC_RIGHT_B_Pin   };

static volatile int32_t s_raw_count[ENC_COUNT];   /* 中断里累加的原始计数 */
static          int32_t s_last_raw[ENC_COUNT];    /* 上一次取走的原始计数 */
static volatile uint8_t s_last_state[ENC_COUNT];  /* 上一次 (A,B) 状态, bit0=A bit1=B */
static Encoder_t        s_enc[ENC_COUNT];

/* ======================== 私有函数 ======================== */

/**
 * @brief  两个定时器配置为 CH3 双边沿输入捕获
 * @note   A 相引脚复用与 B 相普通输入, 都由 tim.c 的 HAL_TIM_IC_MspInit 配置
 */
static void Enc_TimerInit(void)
{
    TIM_IC_InitTypeDef ic = {0};
    uint8_t i;

    ic.ICPolarity  = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;  /* 上升沿 + 下降沿都捕获 */
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;           /* 直连 TI3 */
    ic.ICPrescaler = TIM_ICPSC_DIV1;                     /* 每个边沿都计数 */
    ic.ICFilter    = 0x4;                                /* 滤除 4 个采样周期的毛刺 */

    for (i = 0; i < ENC_COUNT; i++) {
        htim_enc[i].Instance               = (TIM_TypeDef *)s_inst[i];
        htim_enc[i].Init.Prescaler         = 0;
        htim_enc[i].Init.CounterMode       = TIM_COUNTERMODE_UP;
        htim_enc[i].Init.Period            = 0xFFFF;
        htim_enc[i].Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
        htim_enc[i].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

        if (HAL_TIM_IC_Init(&htim_enc[i]) != HAL_OK) {
            Error_Handler();
        }
        if (HAL_TIM_IC_ConfigChannel(&htim_enc[i], &ic, s_channel[i]) != HAL_OK) {
            Error_Handler();
        }
    }
}

/**
 * @brief  清标志、开 NVIC、启动捕获
 */
static void Enc_Start(void)
{
    uint8_t i;

    __HAL_TIM_SET_COUNTER(&htim_enc[ENC_LEFT],  0);
    __HAL_TIM_SET_COUNTER(&htim_enc[ENC_RIGHT], 0);

    __HAL_TIM_CLEAR_FLAG(&htim_enc[ENC_LEFT],  TIM_FLAG_CC3);
    __HAL_TIM_CLEAR_FLAG(&htim_enc[ENC_RIGHT], TIM_FLAG_CC3);

    HAL_NVIC_SetPriority(ENC_LEFT_IRQn,  1, 0);
    HAL_NVIC_EnableIRQ(ENC_LEFT_IRQn);
    HAL_NVIC_SetPriority(ENC_RIGHT_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ENC_RIGHT_IRQn);

    for (i = 0; i < ENC_COUNT; i++) {
        if (HAL_TIM_IC_Start_IT(&htim_enc[i], s_channel[i]) != HAL_OK) {
            Error_Handler();
        }
    }
}

/* ======================== 公有函数 ======================== */

void Encoder_Init(void)
{
    uint8_t i;

    for (i = 0; i < ENC_COUNT; i++) {
        s_raw_count[i] = 0;
        s_last_raw[i]  = 0;
        s_enc[i].count        = 0;
        s_enc[i].delta        = 0;
        s_enc[i].distance_m   = 0.0f;
        s_enc[i].velocity_mps = 0.0f;
        s_enc[i].dir_sign     = 1;
        s_enc[i].io_error     = 0;

        /* 用引脚当前的真实电平初始化状态机, 避免首次中断误判一步 */
        {
            uint8_t a = ((s_a_port[i]->IDR & s_a_pin[i]) != 0u) ? 1u : 0u;
            uint8_t b = ((s_b_port[i]->IDR & s_b_pin[i]) != 0u) ? 1u : 0u;
            s_last_state[i] = (uint8_t)(a | (uint8_t)(b << 1));
        }
    }

    Enc_TimerInit();
    Enc_Start();
}

/**
 * @brief  编码器自检: 由软件直接翻转 A 相引脚, 验证整条通路
 * @param  id       要自检的轮子
 * @param  toggles  翻转次数 (建议 200)
 * @retval 自检期间累计到的计数值
 *
 * @note   这个测试把"定时器捕获 -> NVIC -> 中断 -> 计数"整条链路走一遍,
 *         完全不依赖外部编码器信号, 因此可以把两类问题彻底分开:
 *
 *           返回 ≈ toggles/2 (方向正负不论)  -> 代码通路正常, 问题在外面信号
 *           返回 0                            -> 代码通路有问题, 与硬件无关
 *
 *         做法: 直接把 A 相引脚在 GPIO 输出/复用 之间切换, 制造上升沿和下降沿。
 *         每次翻转之间等 100us, 远大于输入滤波器(N=4, fDTS/2)的 190ns,
 *         不会被滤掉。
 */
int32_t Encoder_SelfTest(EncoderID_t id, uint16_t toggles)
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       af;
    int32_t       before;
    int32_t       after;
    uint16_t      i;
    GPIO_InitTypeDef gpio = {0};

    if ((uint8_t)id >= ENC_COUNT) {
        return 0;
    }

    if (id == ENC_LEFT) {
        port = ENC_LEFT_A_Port;
        pin  = ENC_LEFT_A_Pin;
        af   = GPIO_AF1_TIM1;
    } else {
        port = ENC_RIGHT_A_Port;
        pin  = ENC_RIGHT_A_Pin;
        af   = GPIO_AF1_TIM2;
    }

    /* 先切回普通 GPIO 输出, 这样软件能直接驱动这根线 */
    gpio.Pin   = pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(port, &gpio);

    __disable_irq();
    before = s_raw_count[id];
    __enable_irq();

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    delay_us(100);

    for (i = 0; i < toggles; i++) {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
        delay_us(100);
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
        delay_us(100);
    }

    __disable_irq();
    after = s_raw_count[id];
    __enable_irq();

    /* 测完把引脚还原成定时器捕获输入 */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Alternate = af;
    HAL_GPIO_Init(port, &gpio);

    return (after - before);
}

/**
 * @brief  输入捕获中断处理, 由 TIM1_CC_IRQHandler / TIM2_IRQHandler 调用
 * @note   没有走 HAL_TIM_IC_CaptureCallback, 是为了省掉 HAL_TIM_IRQHandler
 *         里那一长串分支判断, 把单次中断压到几十个周期
 *
 *         判方向用的是"2 位正交状态机", 而不是简单地读 B 相电平。
 *         A 相每来一个边沿, 取 (A,B) 组成 2 位状态, 和上一次状态比较:
 *
 *           只变 1 位            -> 物理上唯一可能的一步, 一定计数
 *           变 2 位(B 相也跳了) -> 按实际发生的那次跳变算, 通常发生在
 *                                  真实换向、A/B 同时变化的时刻
 *           0 位                 -> 没变化, 忽略
 *
 *         比"读 B 相电平"强在哪: 它把状态当作一个整体看待, 单次采样出错
 *         时不会直接翻转方向, 对 A 相毛刺的容忍度明显更高。
 *
 *         状态编码: bit0 = A, bit1 = B
 */
void Encoder_CaptureIRQ(TIM_TypeDef *inst)
{
    uint8_t id;
    uint8_t a_level;
    uint8_t b_level;
    uint8_t code;
    uint8_t prev;
    uint8_t diff;
    uint8_t forward;

    if (inst == ENC_LEFT_TIM) {
        id = ENC_LEFT;
    } else if (inst == ENC_RIGHT_TIM) {
        id = ENC_RIGHT;
    } else {
        return;
    }

    if (__HAL_TIM_GET_FLAG(&htim_enc[id], TIM_FLAG_CC3) == RESET) {
        return;
    }
    __HAL_TIM_CLEAR_FLAG(&htim_enc[id], TIM_FLAG_CC3);

    a_level = ((s_a_port[id]->IDR & s_a_pin[id]) != 0u) ? 1u : 0u;
    b_level = ((s_b_port[id]->IDR & s_b_pin[id]) != 0u) ? 1u : 0u;

    code = (uint8_t)(a_level | (uint8_t)(b_level << 1));
    prev = s_last_state[id];
    s_last_state[id] = code;

    diff = (uint8_t)(code ^ prev);
    if (diff == 0u) {
        return;                    /* 状态没变, 当成抖动 */
    }

    /* 物理上 A 跳变时, (A,B) 只会按 00->01->11->10->00 这个序列走。
       用"新状态对应物理路径上属于上一次状态的下一个"来定方向。 */
    switch (prev) {
        case 0x0: forward = (code == 0x1) ? 1u : 0u; break;   /* 00 */
        case 0x1: forward = (code == 0x3) ? 1u : 0u; break;   /* 01 */
        case 0x3: forward = (code == 0x2) ? 1u : 0u; break;   /* 11 */
        case 0x2: forward = (code == 0x0) ? 1u : 0u; break;   /* 10 */
        default:  forward = (a_level != b_level) ? 1u : 0u; break;
    }

    if (forward != 0u) {
        s_raw_count[id]++;
    } else {
        s_raw_count[id]--;
    }
}

void Encoder_Update(void)
{
    uint8_t i;

    for (i = 0; i < ENC_COUNT; i++) {
        int32_t now;
        int32_t raw;
        int32_t d_signed;

        /* 原子取走原始计数, 避免中断在读取过程中改写 */
        __disable_irq();
        raw = s_raw_count[i];
        __enable_irq();

        /* 原始增量 */
        d_signed = raw - s_last_raw[i];
        s_last_raw[i] = raw;

        /* 应用方向修正 —— 必须只在这里做一次, 后面 count / delta / distance /
           velocity 全部用这个已修正值, 否则会出现 "计数是正的、距离是负的"
           这种自相矛盾的结果 */
        d_signed *= (int32_t)s_enc[i].dir_sign;

        s_enc[i].delta        = (int16_t)d_signed;
        now                   = s_enc[i].count + d_signed;
        s_enc[i].count        = now;
        s_enc[i].distance_m   = (float)now * ENC_METERS_PER_COUNT;
        s_enc[i].velocity_mps = ((float)d_signed * ENC_METERS_PER_COUNT) / ENC_SAMPLE_PERIOD_S;
    }
}

void Encoder_Reset(void)
{
    uint8_t i;

    __disable_irq();
    for (i = 0; i < ENC_COUNT; i++) {
        s_raw_count[i] = 0;
        s_last_raw[i]  = 0;
    }
    __enable_irq();

    for (i = 0; i < ENC_COUNT; i++) {
        s_enc[i].count        = 0;
        s_enc[i].delta        = 0;
        s_enc[i].distance_m   = 0.0f;
        s_enc[i].velocity_mps = 0.0f;
    }
}

const Encoder_t *Encoder_Get(EncoderID_t id)
{
    if ((uint8_t)id >= ENC_COUNT) {
        return 0;
    }
    return &s_enc[id];
}

int16_t Encoder_GetDelta(EncoderID_t id)
{
    if ((uint8_t)id >= ENC_COUNT) {
        return 0;
    }
    return s_enc[id].delta;
}

int32_t Encoder_GetCount(EncoderID_t id)
{
    if ((uint8_t)id >= ENC_COUNT) {
        return 0;
    }
    return s_enc[id].count;
}

/**
 * @brief  取走并清零原始中断计数 (仅用于诊断)
 * @note   返回自上次调用以来捕获中断触发的次数, 不带方向、不带符号修正。
 *         轮子静止时这个值应该稳定为 0; 若持续增长, 说明输入引脚上有干扰,
 *         或者编码器输出悬空。
 */
int32_t Encoder_TakeIsrCount(EncoderID_t id)
{
    int32_t n;

    if ((uint8_t)id >= ENC_COUNT) {
        return 0;
    }

    __disable_irq();
    n = s_raw_count[id];
    s_raw_count[id] = 0;
    __enable_irq();

    return n;
}

void Encoder_SetDirSign(EncoderID_t id, int8_t sign)
{
    if (((uint8_t)id >= ENC_COUNT) || (sign == 0)) {
        return;
    }
    s_enc[id].dir_sign = (sign > 0) ? 1 : -1;
}
