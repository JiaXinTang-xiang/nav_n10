/**
 * @file    TB6612.h
 * @brief   TB6612FNG 双H桥电机驱动 (两轮小车)
 * @note    硬件连接:
 *          - Motor A 方向: AIN1(PE14), AIN2(PE15)
 *          - Motor B 方向: BIN1(PD8),  BIIN2(PD9)
 *          - Motor A PWM:   TIM3_CH1 (PB4)
 *          - Motor B PWM:   TIM3_CH4 (PB1)
 */

#ifndef TB6612_H
#define TB6612_H

#include "main.h"

/* ======================== PWM 限幅默认值 ======================== */
#define TB6612_PWM_MAX   100
#define TB6612_PWM_MIN  (-100)

/* ======================== 电机编号 ======================== */
typedef enum {
    MOTOR_A = 0,
    MOTOR_B = 1,
    MOTOR_COUNT
} MotorID_t;

/* ======================== 函数声明 ======================== */

void TB6612_Init(void);
void Load(int moto1, int moto2);
void Motor_Set(MotorID_t motor, int pwm);
void Motor_Brake(MotorID_t motor);
void Motor_BrakeAll(void);
void Motor_SetLimit(int max, int min);
void Limit(int *motoA, int *motoB);

#endif /* TB6612_H */
