#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "stm32f10x.h"

/* ========== 编码器与车轮参数 (根据实际硬件调整) ========== */
#define ENCODER_PPR         11      // 编码器线数 (电机轴每转脉冲数)
#define ENCODER_MULTIPLIER  4       // 4倍频 (TIM编码器模式TI12)
#define GEAR_RATIO          48      // 电机减速比
#define WHEEL_DIAMETER_MM   65.0f   // 轮径 (mm)
#define WHEEL_TRACK_MM      150.0f  // 两轮轮距 (mm)

/* 自动计算 */
#define PULSE_PER_WHEEL_REV  (ENCODER_PPR * ENCODER_MULTIPLIER * GEAR_RATIO)
#define WHEEL_CIRCUMFERENCE  (3.1415926f * WHEEL_DIAMETER_MM)
#define MM_PER_COUNT         (WHEEL_CIRCUMFERENCE / PULSE_PER_WHEEL_REV)

/* ========== 速度PID默认参数 ========== */
#define SPEED_KP  0.15f
#define SPEED_KI  0.01f
#define SPEED_KD  0.0f
#define SPEED_MAX_OUT   100.0f
#define SPEED_MAX_IOUT  20.0f

void Chassis_Init(void);                                      // 底盘初始化
void Chassis_SetVelocity(float v_linear, float v_angular);    // 设置目标速度 (mm/s, mrad/s)
void Chassis_Update(void);                                    // 10ms周期: 读编码器+PID+里程计
void Chassis_Stop(void);                                      // 急停
void Chassis_GetOdometry(float *x, float *y, float *theta);   // 获取里程计 (mm, mm, mrad)

#endif
