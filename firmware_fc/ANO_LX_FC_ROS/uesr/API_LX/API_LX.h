#ifndef _API_LX_H_
#define _API_LX_H_

#include "stm32f4xx.h"
#include "ANO_DT_LX.h"
#include "LX_FC_Fun.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// 添加Vector2结构体定义
typedef struct {
    float x;
    float y;
} Vector2;

// 无人机状态结构
typedef struct {
    Vector2 position;      // 当前位置
    Vector2 velocity;      // 当前速度
    Vector2* waypoints;    // 路径点数组
    int waypoint_count;    // 路径点数量
    int current_waypoint;  // 当前目标路径点索引
    int hover_start;   		 // 开始悬停的时间
    int is_hovering;       // 是否正在悬停
    int hover_complete;    // 悬停是否完成
}Drone;



u8 API_Take_Off(u16 dt,u16 height_cm);
u8 API_landing(u16 dt);
void Reset_vel(void);
uint8_t API_User_Delay(uint32_t dt);


#endif


