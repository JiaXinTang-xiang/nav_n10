#ifndef XUNJI_TASK_H
#define XUNJI_TASK_H

#include "bsp.h"

extern int   RRR, RR, R, MR, ML, L, LL, LLL;
extern float Xunji_Left, Xunji_Right;
extern int32_t left_pwm, right_pwm;
extern int32_t LeftOut, RightOut;
extern int Xunji_State;
int  sensorPID(void);         /* 读8路红外查表, 返回偏差值 */

/*
 * 传入基础速度, 计算出差速后的左右目标速度
 * l_in/r_in: 基础速度(脉冲/10ms)
 * l_out/r_out: 叠加差速后的速度
 */
void xunji(int32_t l_in, int32_t r_in, int32_t *l_out, int32_t *r_out);
void xunji_PID(int32_t l_in, int32_t r_in, int32_t *l_out, int32_t *r_out);

/**
 * @brief 弯道检测(sensorPID之后调用)
 * @retval -1=左直角  0=无  1=右直角
 */
int Corner_Check(void);

#endif
