#ifndef __IMU_H
#define __IMU_H

#include <math.h>
#define M_PI  (float)3.1415926535
typedef struct
{
    float x;
    float y;
    float z;
} xyz_f_t;
extern xyz_f_t north,west;
extern volatile float yaw[5];   //处理航向的增值
extern float motion6[7];
//Mini IMU AHRS 解算的API
void IMU_init(void); //初始化
void IMU_getYawPitchRoll(float * ypr); //更新姿态
/* 最近一次 IMU_getYawPitchRoll() 采样得到的补偿后 Z 轴角速度，单位 rad/s。 */
float IMU_GetYawRateRadps(void);
/* 最近一次姿态解算的原始 yaw，单位度，范围 [-180,180]。 */
float IMU_GetYawDeg(void);
void IMU_TT_getgyro(float * zsjganda);
//uint32_t micros(void);	//读取系统上电后的时间  单位 us
void MPU6050_InitAng_Offset(void);
#endif

//------------------End of File----------------------------
