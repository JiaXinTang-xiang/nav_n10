#ifndef _CoordinateSystemCalibration_H_
#define _CoordinateSystemCalibration_H_

#include "stm32f4xx.h"
#include "AnoDTRasp.h"
#include "ANO_DT_LX.h"
#include <math.h>  
/*
*                   坐标系校准 
*	SLAM定位的位置数据会随着飞机的摆放初始位置不一样时，
* 获取的数据坐标系与飞机的默认坐标系可能不一样，
* 通过校准后，将SLAM坐标系与飞机坐标系保持一致
*/

typedef struct
{
	int8_t X_Coeff;        /* x轴系数 */
  int8_t Y_Coeff;        /* y轴系数 */
	int8_t X_exchange_Y;   /* 坐标对换 */
}Aircraft_Nose_st;       /* 飞机机头 */

typedef struct
{
	int8_t X_Coeff;        /* x轴系数 */
  int8_t Y_Coeff;        /* y轴系数 */
	int8_t X_exchange_Y;   /* 坐标对换 */
}Aircraft_Tail_st;       /* 飞机机尾 */


typedef struct
{
	int8_t X_Coeff;         /* x轴系数 */
  int8_t Y_Coeff;         /* y轴系数 */
	int8_t X_exchange_Y;    /* 坐标对换 */
}Aircraft_Left_st;        /* 飞机左侧 */

typedef struct
{
	int8_t X_Coeff;    			/* x轴系数 */
  int8_t Y_Coeff;    		  /* y轴系数 */
	int8_t X_exchange_Y;    /* 坐标对换 */
}Aircraft_Right_st;       /* 飞机右侧 */

typedef struct
{
	Aircraft_Nose_st Nose;    /* 飞机机头 */
	Aircraft_Tail_st Tail;    /* 飞机机尾 */
	Aircraft_Left_st Left;    /* 飞机左侧 */
	Aircraft_Right_st Right;  /* 飞机右侧 */
}Aircraft_st;


/* 飞机朝向枚举定义 */
typedef enum {
    DIRECTION_NOSE = 0,  // 机头朝前
    DIRECTION_RIGHT = 1, // 右侧朝前
    DIRECTION_TAIL = 2,  // 机尾朝前
    DIRECTION_LEFT = 3   // 左侧朝前
} AircraftDirection_e;

extern Aircraft_st Aircraft;

/* 函数声明 */
void Aircraft_SLAM_Coeff_Init(void);
uint8_t Calibration_Criteria(void);
void Aircraft_SLAM_Calibration(void);

void ConvertROSToAircraftFrame(float *ros_x, float *ros_y, float ros_yaw);
uint8_t GetAircraftDirection(void);

#endif


