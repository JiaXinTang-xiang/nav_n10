/**
 * @file       example.c
 * @brief      PID控制器简单用法示例
 * @note       演示如何初始化和使用PID控制器
 */

#include "pid.h"

// 定义PID结构体
pid_type_def my_pid;

// PID参数：比例系数Kp=1.0, 积分系数Ki=0.1, 微分系数Kd=0.05
fp32 pid_params[3] = {1.0f, 0.1f, 0.05f};

// 模拟控制过程
fp32 设定值 = 100.0f; // 目标设定值
fp32 反馈值 = 0.0f;   // 当前反馈值
fp32 输出值 = 0.0f;   // PID输出值

int main()
{
    // 初始化PID控制器
    // 模式：PID_POSITION (位置式PID)
    // 最大输出：10.0
    // 最大积分输出：5.0
    PID_init(&my_pid, PID_POSITION, pid_params, 10.0f, 5.0f);

    // 计算PID输出
    输出值 = PID_calc(&my_pid, 反馈值, 设定值);

    // 清除PID状态（可选）
    PID_clear(&my_pid);
}