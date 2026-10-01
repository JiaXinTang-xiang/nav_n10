#ifndef __GET_DATA_H
#define __GET_DATA_H

#include "stdint.h"

//extern double yaw_fix,yaw_zore,yaw_part,yaw_last,yaw_round;//imu输出的欧拉角，单位度
//extern float rol,pit,yaw;
extern double rol,pit,yaw,yaw_fix,yaw_zore,yaw_part,yaw_last,yaw_round;

void get_data(void);

#endif
