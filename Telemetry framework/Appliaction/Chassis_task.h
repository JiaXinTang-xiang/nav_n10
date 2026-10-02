/**
 * @file    Chassis_task.h
 * @brief   底盘任务: 轮速归一化 + 差速运动学 + 左右轮速度闭环 + 里程计
 * @note    符号约定 (只在这一层做统一, 底层 Load() 的负值前进语义不外泄):
 *            - 轮速: 正 = 前进, 单位 m/s
 *            - 里程计: x 增加 = 前进, theta 增加 = 左转(逆时针)
 */

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "main.h"
#include "encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 里程计结构 ======================== */
typedef struct {
    float x_m;        /* 累计 x (米) */
    float y_m;        /* 累计 y (米) */
    float theta_rad;  /* 累计航向 (弧度, 逆时针为正) */
    float v_mps;      /* 线速度 (米/秒) */
    float w_radps;    /* 角速度 (弧度/秒) */
} Chassis_Odom_t;

/* ======================== 接口 ======================== */

/**
 * @brief  底盘初始化 (编码器 + 速度环 PID)
 * @note   必须在 MX_TIM1_Init / MX_TIM2_Init 之后调用
 */
void Chassis_Init(void);

/**
 * @brief  200Hz 周期任务: 读编码器 -> 速度环 -> 输出 PWM -> 更新里程计
 * @note   由 Timer_task.c 的 Task_200Hz 槽位调用 (5ms 周期)
 */
void Chassis_Update(void);

/**
 * @brief  设置左右目标轮速 (归一化接口, 正 = 前进)
 * @param  left_mps   左轮目标线速度 (米/秒)
 * @param  right_mps  右轮目标线速度 (米/秒)
 */
void Chassis_SetWheelSpeed(float left_mps, float right_mps);

/**
 * @brief  直接给左右轮 PWM (调试用, 正 = 前进, 范围 ±100)
 * @note   会同时把速度环目标清零, 避免 PID 在背后继续动
 */
void Chassis_SetWheelRaw(int left, int right);

/**
 * @brief  差速运动学逆解: 整车线速度/角速度 -> 左右轮速
 * @param  v_mps    线速度 (米/秒)
 * @param  w_radps  角速度 (弧度/秒)
 */
void Chassis_SetTwist(float v_mps, float w_radps);

/**
 * @brief  停车 (滑行)
 */
void Chassis_Stop(void);

/**
 * @brief  刹车 (H 桥短接, 快速停住)
 */
void Chassis_Brake(void);

/**
 * @brief  取里程计 (只读)
 */
const Chassis_Odom_t *Chassis_GetOdom(void);

/**
 * @brief  里程计清零
 */
void Chassis_ResetOdom(void);

/**
 * @brief  左右轮目标速度 (只读)
 */
float Chassis_GetTarget(EncoderID_t wheel);

/**
 * @brief  上电自检: 软件翻转 A 相引脚, 验证捕获通路
 * @note   在 Chassis_Init() 之后调用。全程约 80ms, 期间会占用两个 A 相引脚,
 *         自检完自动还原为捕获输入。结果用 Chassis_DebugDisplaySelfTest() 查看。
 */
void Chassis_SelfTest(void);

/**
 * @brief  显示自检结果
 * @note   期望值: L/R 都约等于 +100 (A!=B 判为正转)
 */
void Chassis_DebugDisplaySelfTest(void);

/**
 * @brief  调试用: 在 OLED 上显示左右轮计数/速度/里程计
 * @note   标定阶段专用, 标定完成后可以从主循环里去掉
 */
void Chassis_DebugDisplay(void);

/**
 * @brief  诊断显示: 显示引脚上的原始捕获中断频率 (Hz) + 累计计数
 * @note   用来区分"信号太密/有干扰"和"计数逻辑错":
 *           轮子静止        -> ISR 应该为 0
 *           手转 1 圈/秒    -> ISR 应该约 26 Hz
 *           扭一下突然上万   -> 看 ISR 是不是几百 kHz
 */
void Chassis_DebugDisplayIsr(void);

/**
 * @brief  速度环测试: 两轮目标速度设为 +0.2 m/s 闭环运行
 */
void Chassis_TestSpeedStart(void);

/**
 * @brief  开环基准测试: 用 Load(-20,-20) 那个 PWM 值直接跑
 * @note   用来和闭环对比: 同样的"舒服速度"到底需要多少 PWM
 */
void Chassis_TestOpenLoopStart(void);

/**
 * @brief  速度环测试: 停止并回到开环零输出
 */
void Chassis_TestStop(void);

/**
 * @brief  显示速度环测试
 * @note   行 1: 目标轮速 (mm/s, 左,右)
 *         行 2: 实测轮速 (mm/s), 应稳定跟住目标
 *         行 3: 累计计数
 *         行 4: PID 输出 (PWM 占空比绝对值)
 */
void Chassis_DebugDisplaySpeedTest(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_TASK_H */
