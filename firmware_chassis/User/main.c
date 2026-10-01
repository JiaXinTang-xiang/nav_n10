#include "bsp.h"
#include "Serial.h"
#include "KEY.h"

int Run = 0;

extern float cmd_v_linear;
extern float cmd_v_angular;

int main(void)
{
    Timer_Init();        // TIM1 10ms中断
    OLED_Init();
    Serial_Init();       // USART1 Jetson通信
    Key_Init();
    Chassis_Init();      // 电机+编码器+速度PID

    OLED_ShowString(1, 1, "Chassis Ready");
    Delay_s(1);

    while (1)
    {
        /* Jetson速度指令 (0xBB协议) */
        if (Serial_GetCmdVelFlag())
        {
            Chassis_SetVelocity(cmd_v_linear, cmd_v_angular);
        }

        /* 视觉数据 (0xAA协议, 保留) */
        if (Serial_GetRxFlag())
        {
            uint16_t cx, cy, ar;
            Serial_GetData(&cx, &cy, &ar);
        }

        /* OLED: 目标速度 + 里程计 */
        float ox, oy, ot;
        Chassis_GetOdometry(&ox, &oy, &ot);
        OLED_ShowSignedNum(1, 1, (int16_t)cmd_v_linear,  5);
        OLED_ShowSignedNum(2, 1, (int16_t)cmd_v_angular, 5);
        OLED_ShowSignedNum(3, 1, (int16_t)ox, 5);
        OLED_ShowSignedNum(4, 1, (int16_t)oy, 5);
    }
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        static uint8_t odom_cnt = 0;

        Key_Tick();
        Chassis_Update();

        /* 每50ms回传里程计 */
        if (++odom_cnt >= 5)
        {
            odom_cnt = 0;
            float x, y, theta;
            Chassis_GetOdometry(&x, &y, &theta);
            Serial_SendOdometry(x, y, theta / 1000.0f);
        }

        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
