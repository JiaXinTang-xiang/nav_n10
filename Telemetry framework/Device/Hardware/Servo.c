/**
 * @file    Servo.c
 * @brief   SG90 舵机驱动模块实现 — 四路 PWM 舵机控制
 * @note    基于 STM32F4 HAL 库, 使用 TIM4 的四路 PWM 输出通道
 *          控制 4 个 SG90 舵机 (SG901~SG904)
 * @author  小雨
 * @date    2026-07-27
 */

#include "Servo.h"
#include "tim.h"        /* 引用外部句柄 htim4 */

/* ================================================================
 *                        模块内部静态变量
 * ================================================================ */

/** @brief 四路舵机当前角度记录 (0° ~ 180°) */
static uint8_t servo_current_angle[4] = {90, 90, 90, 90};

/* ================================================================
 *                      内部辅助函数
 * ================================================================ */

/**
 * @brief  将用户通道编号转换为 HAL 库的 TIM_CHANNEL_x 宏
 * @param  ch  用户通道编号 (1~4)
 * @retval HAL 通道宏 (TIM_CHANNEL_1 ~ TIM_CHANNEL_4), 非法值返回 0
 */
static uint32_t Servo_ChannelToTIM(uint8_t ch)
{
    switch (ch)
    {
        case SERVO_CH_1:  return TIM_CHANNEL_1;
        case SERVO_CH_2:  return TIM_CHANNEL_2;
        case SERVO_CH_3:  return TIM_CHANNEL_3;
        case SERVO_CH_4:  return TIM_CHANNEL_4;
        default:          return 0;   /* 非法通道 */
    }
}

/**
 * @brief  将角度值转换为 PWM 脉冲宽度 (CCR 值)
 * @param  angle  角度 (0° ~ 180°)
 * @note   换算公式推导:
 *             脉冲范围 = 2500 - 500 = 2000 tick
 *             角度范围 = 180° - 0° = 180°
 *             斜率 = 2000 / 180 ≈ 11.11 tick/°
 *             pulse = 500 + angle * 2000 / 180
 * @retval 脉冲宽度 (timer tick 数)
 */
static uint16_t Servo_AngleToPulse(uint8_t angle)
{
    uint32_t pulse;

    /* 限幅保护: 确保角度在 [0, 180] 范围内 */
    if (angle > SERVO_ANGLE_MAX) {
        angle = SERVO_ANGLE_MAX;
    }

    /**
     * 线性映射: pulse = MIN + angle × RANGE / 180
     * 使用 uint32_t 中间变量, 避免 16 位乘法溢出
     * 例: angle=90 → pulse = 500 + 90×2000/180 = 500 + 1000 = 1500 (1.5ms)
     */
    pulse = SERVO_MIN_PULSE + ((uint32_t)angle * SERVO_PULSE_RANGE) / SERVO_ANGLE_MAX;

    return (uint16_t)pulse;
}

/**
 * @brief  验证通道编号是否合法
 * @param  ch  用户通道编号
 * @retval true  合法通道
 * @retval false 非法通道
 */
static bool Servo_IsValidChannel(uint8_t ch)
{
    return (ch >= SERVO_CH_1 && ch <= SERVO_CH_4);
}

/* ================================================================
 *                        API 函数实现
 * ================================================================ */

/**
 * @brief  舵机模块初始化
 * @note   启动 TIM4 全部四路 PWM 并使所有舵机归中 (90°)
 *         TIM4 的时基和 GPIO 已由 CubeMX 生成的 MX_TIM4_Init() 完成配置,
 *         此处仅启动 PWM 输出并设置初始占空比
 */
void Servo_Init(void)
{
    /* 启动四路 PWM 输出通道 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);   /* SG901 — PD12 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);   /* SG902 — PD13 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);   /* SG903 — PD14 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);   /* SG904 — PD15 */

    /* 所有舵机初始归中 (90°, 对应脉冲 1.5ms = 1500 tick) */
    Servo_SetAngle(SERVO_CH_1, 90);
    Servo_SetAngle(SERVO_CH_2, 90);
    Servo_SetAngle(SERVO_CH_3, 90);
    Servo_SetAngle(SERVO_CH_4, 90);
}

/**
 * @brief  舵机模块反初始化
 * @note   停止全部 PWM 输出
 */
void Servo_DeInit(void)
{
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}

/**
 * @brief  启动单个舵机通道
 */
void Servo_Start(uint8_t ch)
{
    uint32_t tim_ch = Servo_ChannelToTIM(ch);
    if (tim_ch != 0) {
        HAL_TIM_PWM_Start(&htim4, tim_ch);
    }
}

/**
 * @brief  停止单个舵机通道
 */
void Servo_Stop(uint8_t ch)
{
    uint32_t tim_ch = Servo_ChannelToTIM(ch);
    if (tim_ch != 0) {
        HAL_TIM_PWM_Stop(&htim4, tim_ch);
    }
}

/**
 * @brief  设置舵机角度 (立即跳转)
 * @note   使用 __HAL_TIM_SET_COMPARE 宏直接写 CCR 寄存器,
 *         比 HAL_TIM_PWM_ConfigChannel 效率更高, 适合频繁更新
 */
void Servo_SetAngle(uint8_t ch, uint8_t angle)
{
    uint32_t tim_ch;
    uint16_t pulse;

    /* 通道合法性检查 */
    if (!Servo_IsValidChannel(ch)) {
        return;
    }

    /* 角度限幅 */
    if (angle > SERVO_ANGLE_MAX) {
        angle = SERVO_ANGLE_MAX;
    }

    /* 记录当前角度 */
    servo_current_angle[ch - 1] = angle;

    /* 角度 → 脉冲值转换 */
    pulse = Servo_AngleToPulse(angle);

    /* 通道枚举 → HAL 通道宏 */
    tim_ch = Servo_ChannelToTIM(ch);

    /* 写入 CCR 寄存器, 立即更新占空比 */
    __HAL_TIM_SET_COMPARE(&htim4, tim_ch, pulse);
}

/**
 * @brief  设置舵机原始脉冲宽度
 * @note   直接操作 CCR 值, 适用于非标准角度范围的场景
 */
void Servo_SetPulse(uint8_t ch, uint16_t pulse)
{
    uint32_t tim_ch;

    if (!Servo_IsValidChannel(ch)) {
        return;
    }

    /* 脉冲限幅保护 */
    if (pulse < SERVO_MIN_PULSE) {
        pulse = SERVO_MIN_PULSE;
    }
    if (pulse > SERVO_MAX_PULSE) {
        pulse = SERVO_MAX_PULSE;
    }

    tim_ch = Servo_ChannelToTIM(ch);

    /**
     * 从脉冲值反推角度并记录 (用于 Servo_GetAngle 查询)
     * 反推公式: angle = (pulse - 500) * 180 / 2000
     */
    servo_current_angle[ch - 1] = (uint8_t)(
        ((uint32_t)(pulse - SERVO_MIN_PULSE) * SERVO_ANGLE_MAX) / SERVO_PULSE_RANGE
    );

    __HAL_TIM_SET_COMPARE(&htim4, tim_ch, pulse);
}

/**
 * @brief  获取舵机当前角度
 */
uint8_t Servo_GetAngle(uint8_t ch)
{
    if (!Servo_IsValidChannel(ch)) {
        return 0;
    }
    return servo_current_angle[ch - 1];
}

/**
 * @brief  舵机平滑运动 (线性插值)
 * @note   算法步骤:
 *          1. 计算总步数: steps = duration_ms / 20ms
 *          2. 计算每步角度增量: step_angle = (target - current) / steps
 *          3. 逐步更新角度, 每步间隔 20ms (delay_ms)
 *        优点: 运动平滑, 减少舵机冲击电流
 *        缺点: 阻塞式延时, 运动期间占用 CPU
 */
void Servo_SmoothMove(uint8_t ch, uint8_t target_angle, uint16_t duration_ms)
{
    uint16_t steps;             /* 总步数 */
    float    current_angle_f;   /* 当前角度 (浮点, 用于精确累加) */
    float    step_increment;    /* 每步角度增量 (可能是小数) */
    uint16_t i;

    /* 参数合法性检查 */
    if (!Servo_IsValidChannel(ch) || duration_ms == 0) {
        return;
    }

    /* 目标角度限幅 */
    if (target_angle > SERVO_ANGLE_MAX) {
        target_angle = SERVO_ANGLE_MAX;
    }

    /* 计算总步数 (每步 20ms) */
    steps = duration_ms / SERVO_SMOOTH_STEP_MS;
    if (steps == 0) {
        steps = 1;  /* 最少 1 步 */
    }

    /* 计算每步角度增量 (浮点精度, 避免整数截断累积误差) */
    current_angle_f = (float)servo_current_angle[ch - 1];
    step_increment  = ((float)target_angle - current_angle_f) / (float)steps;

    /**
     * 主循环: 逐步更新角度
     * 每步间隔 20ms (= SERVO_SMOOTH_STEP_MS), 与舵机 PWM 周期一致
     */
    for (i = 0; i < steps; i++)
    {
        current_angle_f += step_increment;

        /* 四舍五入取整, 确保最后一步精确到达目标 */
        uint8_t angle = (uint8_t)(current_angle_f + 0.5f);

        Servo_SetAngle(ch, angle);

        /* 阻塞延时 20ms */
        delay_ms(SERVO_SMOOTH_STEP_MS);
    }

    /* 最后一步: 确保精确到达目标角度 (消除浮点累积误差) */
    Servo_SetAngle(ch, target_angle);
}

/* ================================================================
 *                     main.c 调用示例 (供参考)
 * ================================================================
 *
 *   // -------------------- 初始化 --------------------
 *   // 在 main() 中 MX_TIM4_Init() 之后调用一次即可
 *   Servo_Init();     // 启动四路PWM, 所有舵机归中90°
 *
 *   // -------------------- 基本角度控制 --------------------
 *   Servo_SetAngle(SERVO_CH_1, 0);     // SG901 → 0°
 *   Servo_SetAngle(SERVO_CH_2, 45);    // SG902 → 45°
 *   Servo_SetAngle(SERVO_CH_3, 90);    // SG903 → 90° (居中)
 *   Servo_SetAngle(SERVO_CH_4, 180);   // SG904 → 180°
 *
 *   // -------------------- 平滑运动 --------------------
 *   // SG901 在 500ms 内平滑转到 120°
 *   Servo_SmoothMove(SERVO_CH_1, 120, 500);
 *   // SG902 在 2 秒内平滑转到 60°
 *   Servo_SmoothMove(SERVO_CH_2, 60, 2000);
 *
 *   // -------------------- 查询当前角度 --------------------
 *   uint8_t angle1 = Servo_GetAngle(SERVO_CH_1);  // 读取 SG901 当前角度
 *   uint8_t angle2 = Servo_GetAngle(SERVO_CH_2);
 *
 *   // -------------------- 单通道启停 --------------------
 *   Servo_Stop(SERVO_CH_3);          // 停止 SG903 (PWM输出关闭, 舵机松力)
 *   Servo_Start(SERVO_CH_3);         // 重新启动 SG903
 *   Servo_SetAngle(SERVO_CH_3, 90);  // 归中
 *
 *   // -------------------- 高级: 直接写脉冲 --------------------
 *   Servo_SetPulse(SERVO_CH_1, 1500);  // 1.5ms 脉冲 (90°)
 *   Servo_SetPulse(SERVO_CH_2, 800);   // 0.8ms 脉冲 (~27°)
 *
 *   // -------------------- 反初始化 (停止全部) --------------------
 *   Servo_DeInit();   // 停止全部四路PWM
 *
 *   ============================================================
 *   注意事项:
 *   1. Servo_Init() 必须在 MX_TIM4_Init() 之后调用
 *   2. Servo_SmoothMove() 是阻塞的, 运动期间会占用CPU
 *   3. 角度超出 [0, 180] 会自动限幅, 不会出错
 *   4. 通道编号用 SERVO_CH_1~4 枚举, 不要直接填数字
 *   ============================================================
 */
