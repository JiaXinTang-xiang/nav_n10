/**
 * @file    TB6612.c
 * @brief   TB6612FNG 双H桥电机驱动, 两轮小车
 * @note    TB6612 控制逻辑:
 *          IN1=1 IN2=0 → 正转 | IN1=0 IN2=1 → 反转
 *          IN1=0 IN2=0 → 滑行(自由减速)
 *          IN1=1 IN2=1 → 短刹车(电机绕组短接)
 */

#include "TB6612.h"

/* ======================== 方向引脚控制宏 ======================== */
#define AIN1_WRITE(x) HAL_GPIO_WritePin(AIN1_GPIO_Port,  AIN1_Pin,  (GPIO_PinState)(x))
#define AIN2_WRITE(x) HAL_GPIO_WritePin(AIN2_GPIO_Port,  AIN2_Pin,  (GPIO_PinState)(x))
#define BIN1_WRITE(x) HAL_GPIO_WritePin(BIN1_GPIO_Port,  BIN1_Pin,  (GPIO_PinState)(x))
#define BIN2_WRITE(x) HAL_GPIO_WritePin(BIIN2_GPIO_Port, BIIN2_Pin, (GPIO_PinState)(x))

/* ======================== 私有变量 ======================== */

static int pwm_max = TB6612_PWM_MAX;   /* PWM 正向最大值 */
static int pwm_min = TB6612_PWM_MIN;   /* PWM 反向最大值 (负值) */

extern TIM_HandleTypeDef htim3;         /* TIM3: CH1=MotorA(PB4), CH4=MotorB(PB1) */

/* ======================== 私有函数 ======================== */

/**
 * @brief  PWM 限幅, 将值钳位在 [pwm_min, pwm_max] 范围内
 */
static int Clamp(int pwm)
{
    if (pwm > pwm_max) return pwm_max;
    if (pwm < pwm_min) return pwm_min;
    return pwm;
}

/**
 * @brief  设置电机方向引脚
 * @param  motor  电机编号
 * @param  dir    1=正转, -1=反转, 0=滑行(IN1=IN2=0)
 */
static void SetDirection(MotorID_t motor, int dir)
{
    switch (motor) {
    case MOTOR_A:
        AIN1_WRITE(dir > 0 ? 1 : 0);
        AIN2_WRITE(dir < 0 ? 1 : 0);
        break;
    case MOTOR_B:
        BIN1_WRITE(dir > 0 ? 0 : 1);
        BIN2_WRITE(dir < 0 ? 0 : 1);
        break;
    default:
        break;
    }
}

/**
 * @brief  设置电机 PWM 占空比 (直接写 CCR 寄存器)
 * @param  motor    电机编号
 * @param  compare  比较值 (0 ~ ARR)
 */
static void SetPWM(MotorID_t motor, uint32_t compare)
{
    uint32_t channel;
    switch (motor) {
    case MOTOR_A: channel = TIM_CHANNEL_1; break;   /* TIM3_CH1 (PB4) */
    case MOTOR_B: channel = TIM_CHANNEL_4; break;   /* TIM3_CH4 (PB1) */
    default: return;
    }
    __HAL_TIM_SET_COMPARE(&htim3, channel, compare);
}

/* ======================== 公有函数 ======================== */

/**
 * @brief  TB6612 初始化, 启动 PWM 输出
 * @note   需在 TIM3 初始化完成后调用, PWM 频率 = 10kHz, ARR=99
 */
void TB6612_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   /* Motor A: TIM3_CH1 (PB4) */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);   /* Motor B: TIM3_CH4 (PB1) */
}

/**
 * @brief  对两个电机的 PWM 值进行限幅 (原地修改)
 * @param  motoA  电机 A PWM 值指针
 * @param  motoB  电机 B PWM 值指针
 */
void Limit(int *motoA, int *motoB)
{
    if (motoA != NULL) {
        if (*motoA > pwm_max) *motoA = pwm_max;
        if (*motoA < pwm_min) *motoA = pwm_min;
    }
    if (motoB != NULL) {
        if (*motoB > pwm_max) *motoB = pwm_max;
        if (*motoB < pwm_min) *motoB = pwm_min;
    }
}

/**
 * @brief  两轮差速控制
 * @param  moto1  Motor A PWM, 正值前进 / 负值后退 / 0 滑行
 * @param  moto2  Motor B PWM, 正值前进 / 负值后退 / 0 滑行
 * @note   内部自动限幅, 调用前无需手动 Limit
 * @usage  Load(100, 100)   两轮全速前进
 *         Load(-30, 30)  原地左转 (差速)
 *         Load(0, 0)       滑行停止
 */
void Load(int moto1, int moto2)
{
    moto1 = Clamp(moto1);
    moto2 = Clamp(moto2);

    SetDirection(MOTOR_A, moto1 > 0 ? 1 : (moto1 < 0 ? -1 : 0));
    SetDirection(MOTOR_B, moto2 > 0 ? 1 : (moto2 < 0 ? -1 : 0));

    SetPWM(MOTOR_A, (uint32_t)(moto1 > 0 ? moto1 : -moto1));
    SetPWM(MOTOR_B, (uint32_t)(moto2 > 0 ? moto2 : -moto2));
}

/**
 * @brief  单电机控制
 * @param  motor  电机编号 (MOTOR_A / MOTOR_B)
 * @param  pwm    PWM 值, 正=正转, 负=反转, 0=滑行
 */
void Motor_Set(MotorID_t motor, int pwm)
{
    if (motor >= MOTOR_COUNT) return;
    pwm = Clamp(pwm);
    SetDirection(motor, pwm > 0 ? 1 : (pwm < 0 ? -1 : 0));
    SetPWM(motor, (uint32_t)(pwm > 0 ? pwm : -pwm));
}

/**
 * @brief  单电机刹车 (短路制动)
 * @param  motor  电机编号
 * @note   IN1=IN2=1 → H 桥两个下管 (或上管) 同时导通, 电机绕组短接, 快速制动
 *         区别于 PWM=0 的滑行模式 (IN1=IN2=0, 电机自由减速)
 */
void Motor_Brake(MotorID_t motor)
{
    switch (motor) {
    case MOTOR_A:
        AIN1_WRITE(1); AIN2_WRITE(1);   /* 短刹车: IN1=IN2=1 */
        break;
    case MOTOR_B:
        BIN1_WRITE(1); BIN2_WRITE(1);
        break;
    default:
        break;
    }
    SetPWM(motor, 0);   /* PWM 也清零, 确保安全 */
}

/**
 * @brief  全部电机刹车
 */
void Motor_BrakeAll(void)
{
    Motor_Brake(MOTOR_A);
    Motor_Brake(MOTOR_B);
}

/**
 * @brief  设置 PWM 限幅范围
 * @param  max  最大正值 (>0)
 * @param  min  最大负值 (<0, 应为负数)
 */
void Motor_SetLimit(int max, int min)
{
    if (max > 0) pwm_max = max;
    if (min < 0) pwm_min = min;
}
