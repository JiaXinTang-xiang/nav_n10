#ifndef IMU_TURNCONTROL_H
#define IMU_TURNCONTROL_H

#include "bsp.h"

extern uint8_t Run;             /* 1=正在转弯  0=空闲 */
extern float   target;          /* 目标 yaw 角度 */

void Motor_Turn(float Turn_angle);  /* 设目标 + 启动 */
void IMU_Turn_Run(void);            /* 定时器里调用，跑 PID */
void IMU_Ctrl_Init(void);

#endif /* TB6612_H */



