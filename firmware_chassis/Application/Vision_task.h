#ifndef __VISION_TASK_H
#define __VISION_TASK_H

#include "bsp.h"

extern float Motor_out;	// µç»úÊä³ö

void PID_Ctrl_Init(void);
void Vision_task(void);
int Motor_Turn(float Turn_angle);


#endif

