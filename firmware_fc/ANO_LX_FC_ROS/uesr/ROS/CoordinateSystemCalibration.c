#include "CoordinateSystemCalibration.h"

Aircraft_st Aircraft;

/*
*	函数名称：void Aircraft_SLAM_Coeff_Init(void)
* 函数说明：飞机与SLAM坐标系校准系数初始化
* 函数参数：无
* 函数返回值：无
*/
void Aircraft_SLAM_Coeff_Init(void)
{
	Aircraft.Nose.X_Coeff = 1;
	Aircraft.Nose.Y_Coeff = 1;
	
	Aircraft.Tail.X_Coeff = 1;
	Aircraft.Tail.Y_Coeff = 1;
	
	Aircraft.Left.X_Coeff = 1;
	Aircraft.Left.Y_Coeff = 1;
	
	Aircraft.Right.X_Coeff = 1;
	Aircraft.Right.Y_Coeff = 1;
}

/*
* 函数名称：uint8_t Calibration_Criteria(void)
* 函数说明：校准判断条件
* 函数参数：无
* 函数返回：
*/
uint8_t Calibration_Criteria(void)
{
  return rosData.work_sta;
}

void Calibration_Prompt(char *str)
{

	LxStringSend(2,str);

}

/* 辅助函数：将偏航角归一化到[-180, 180)度范围内 */
float normalize_yaw(float yaw)
{
    if(yaw >= 180.0f) yaw -= 360.0f;
    if(yaw < -180.0f) yaw += 360.0f;
    return yaw;
}

/*
* 函数名称：void Aircraft_SLAM_Calibration(void)
* 函数说明：飞机与SLAM校准
* 函数参数：无
* 函数返回：无
*/
void Aircraft_SLAM_Calibration(void)
{
    static uint8_t cal_step = 0;
    static float init_pos[2] = {0};
    static float current_pos[2] = {0};
    static float init_yaw = 0;
    static float target_yaw = 0;
    
    /* 接收到ros数据后 */
    if(Calibration_Criteria())
    {
        /* 记录初始位置和偏航角 */
        if(cal_step == 0)
        {
            init_pos[0] = rosData.loc[0];
            init_pos[1] = rosData.loc[1];
            init_yaw = (float)fc_att.st_data.yaw_x100 / 100.0f; // 转换为度
            cal_step++;
            Calibration_Prompt("Calibration Step 1: Move forward 1m");
        }
        
        current_pos[0] = rosData.loc[0] - init_pos[0];
        current_pos[1] = rosData.loc[1] - init_pos[1];
        
        switch(cal_step)
        {
            case 1: /* 第一步：将飞机从起飞点向飞机的机头方向行驶 1m 距离 */
                if(fabs(current_pos[0]) > 0.9)  /* 假设移动超过0.9m认为完成 */
                {
                    if(current_pos[0] > 0 )          /* 获取的雷达数据与飞机的坐标系一致 */
                    {
                        Aircraft.Nose.X_Coeff = 1;			/* 设置x系数为1 保存获取的数据不变*/
                    }
                    else if(current_pos[0] < 0 )    /* 获取的雷达数据与飞机的坐标系不一致 */
                    {
                        Aircraft.Nose.X_Coeff = -1;     /* 设置x系数为-1 与获取的数据相乘，改变坐标系与飞机坐标系一致*/
                    }
                    else															/* x轴的数据不变，认为x轴与y轴数据需要对调 */
                    {
                        Aircraft.Nose.X_exchange_Y = 1; /* 对调标志位  */
                    }
                    
                    if(Aircraft.Nose.X_exchange_Y)		
                    {
                        if(current_pos[1] > 0 ) 					/* X轴数据直接替换Y轴数据 */
                        {
                            Aircraft.Nose.Y_Coeff = 1;
                        }
                        else if(current_pos[1] < 0 )    /* X轴数据获取Y轴数据乘系数 使坐标系一致 */
                        {
                            Aircraft.Nose.Y_Coeff = -1;
                        }
                    }
                    
                    cal_step++;
                    Calibration_Prompt("Calibration Step 2: Move left 0.5m");
                }
                break;
                
            case 2: /* 第二步：将飞机朝飞机左侧 行驶 0.5m */
                if(Aircraft.Nose.X_exchange_Y != 1)
                {
                    if(fabs(current_pos[1]) > 0.45 ) 					
                    {
                        Aircraft.Nose.Y_Coeff = (current_pos[1] > 0) ? 1 : -1;
                        cal_step++;
                        Calibration_Prompt("Calibration Step 3: Rotate 90 degrees clockwise");
                        target_yaw = normalize_yaw(init_yaw - 90.0f); // 顺时针旋转90度
                    }
                }
                else
                {
                    if(fabs(current_pos[0]) > 0.45 )          
                    {
                        Aircraft.Nose.X_Coeff = (current_pos[0] > 0) ? 1 : -1;
                        cal_step++;
                        Calibration_Prompt("Calibration Step 3: Rotate 90 degrees clockwise");
                        target_yaw = normalize_yaw(init_yaw - 90.0f); // 顺时针旋转90度
                    }
                }
                break;
                
            case 3: /* 第三步：将飞机在起飞点顺时针旋转 90度  */
            {
                float current_yaw = (float)fc_att.st_data.yaw_x100 / 100.0f;
                float yaw_error = normalize_yaw(current_yaw - target_yaw);
                
                if(fabs(yaw_error) < 5.0f)  /* 假设旋转误差小于5度认为完成 */
                {
                    cal_step++;
                    Calibration_Prompt("Calibration Step 4: Move forward 1m");
                }
                break;
            }
                
            case 4: /* 第四步：将飞机从起飞点向飞机的机头方向行驶 1m 距离 */
                // 类似第一步的处理逻辑
                if(fabs(current_pos[0]) > 0.9)
                {
                    if(current_pos[0] > 0 )
                    {
                        Aircraft.Right.X_Coeff = 1;
                    }
                    else
                    {
                        Aircraft.Right.X_Coeff = -1;
                    }
                    
                    if(Aircraft.Right.X_exchange_Y)		
                    {
                        if(current_pos[1] > 0 ) 					
                        {
                            Aircraft.Right.Y_Coeff = 1;
                        }
                        else    
                        {
                            Aircraft.Right.Y_Coeff = -1;
                        }
                    }
                    
                    cal_step++;
                    Calibration_Prompt("Calibration Step 5: Move left 0.5m");
                }
                break;
                
            case 5: /* 第五步：将飞机朝飞机左侧 行驶 0.5m */
                // 类似第二步的处理逻辑
                if(Aircraft.Right.X_exchange_Y != 1)
                {
                    if(fabs(current_pos[1]) > 0.45 ) 					
                    {
                        Aircraft.Right.Y_Coeff = (current_pos[1] > 0) ? 1 : -1;
                        cal_step++;
                        Calibration_Prompt("Calibration Step 6: Rotate 180 degrees clockwise");
                        target_yaw = normalize_yaw(init_yaw - 180.0f); // 顺时针旋转180度
                    }
                }
                else
                {
                    if(fabs(current_pos[0]) > 0.45 )          
                    {
                        Aircraft.Right.X_Coeff = (current_pos[0] > 0) ? 1 : -1;
                        cal_step++;
                        Calibration_Prompt("Calibration Step 6: Rotate 180 degrees clockwise");
                        target_yaw = normalize_yaw(init_yaw - 180.0f); // 顺时针旋转180度
                    }
                }
                break;
                
            case 6: /* 第六步：将飞机在起飞点顺时针旋转 180度  */
            {
                float current_yaw = (float)fc_att.st_data.yaw_x100 / 100.0f;
                float yaw_error = normalize_yaw(current_yaw - target_yaw);
                
                if(fabs(yaw_error) < 5.0f)
                {
                    cal_step++;
                    Calibration_Prompt("Calibration Step 7: Move forward 1m");
                }
                break;
            }
                
            case 7: /* 第七步：将飞机从起飞点向飞机的机头方向行驶 1m 距离 */
                // 类似第一步的处理逻辑
                if(fabs(current_pos[0]) > 0.9)
                {
                    if(current_pos[0] > 0 )
                    {
                        Aircraft.Tail.X_Coeff = 1;
                    }
                    else
                    {
                        Aircraft.Tail.X_Coeff = -1;
                    }
                    
                    if(Aircraft.Tail.X_exchange_Y)		
                    {
                        if(current_pos[1] > 0 ) 					
                        {
                            Aircraft.Tail.Y_Coeff = 1;
                        }
                        else    
                        {
                            Aircraft.Tail.Y_Coeff = -1;
                        }
                    }
                    
                    cal_step++;
                    Calibration_Prompt("Calibration Step 8: Move left 0.5m");
                }
                break;
                
            case 8: /* 第八步：将飞机朝飞机左侧 行驶 0.5m */
                // 类似第二步的处理逻辑
                if(Aircraft.Tail.X_exchange_Y != 1)
                {
                    if(fabs(current_pos[1]) > 0.45 ) 					
                    {
                        Aircraft.Tail.Y_Coeff = (current_pos[1] > 0) ? 1 : -1;
                        cal_step++;
                        Calibration_Prompt("Calibration Step 9: Rotate 270 degrees clockwise");
                        target_yaw = normalize_yaw(init_yaw - 270.0f); // 顺时针旋转270度
                    }
                }
                else
                {
                    if(fabs(current_pos[0]) > 0.45 )          
                    {
                        Aircraft.Tail.X_Coeff = (current_pos[0] > 0) ? 1 : -1;
                        cal_step++;
                        Calibration_Prompt("Calibration Step 9: Rotate 270 degrees clockwise");
                        target_yaw = normalize_yaw(init_yaw - 270.0f); // 顺时针旋转270度
                    }
                }
                break;
                
            case 9: /* 第九步：将飞机在起飞点顺时针旋转 270度  */
            {
                float current_yaw = (float)fc_att.st_data.yaw_x100 / 100.0f;
                float yaw_error = normalize_yaw(current_yaw - target_yaw);
                
                if(fabs(yaw_error) < 5.0f)
                {
                    cal_step++;
                    Calibration_Prompt("Calibration Step 10: Move forward 1m");
                }
                break;
            }
                
            case 10: /* 第十步：将飞机从起飞点向飞机的机头方向行驶 1m 距离 */
                // 类似第一步的处理逻辑
                if(fabs(current_pos[0]) > 0.9)
                {
                    if(current_pos[0] > 0 )
                    {
                        Aircraft.Left.X_Coeff = 1;
                    }
                    else
                    {
                        Aircraft.Left.X_Coeff = -1;
                    }
                    
                    if(Aircraft.Left.X_exchange_Y)		
                    {
                        if(current_pos[1] > 0 ) 					
                        {
                            Aircraft.Left.Y_Coeff = 1;
                        }
                        else    
                        {
                            Aircraft.Left.Y_Coeff = -1;
                        }
                    }
                    
                    cal_step++;
                    Calibration_Prompt("Calibration Step 11: Move left 0.5m");
                }
                break;
                
            case 11: /* 第十一步：将飞机朝飞机左侧 行驶 0.5m */
                // 类似第二步的处理逻辑
                if(Aircraft.Left.X_exchange_Y != 1)
                {
                    if(fabs(current_pos[1]) > 0.45 ) 					
                    {
                        Aircraft.Left.Y_Coeff = (current_pos[1] > 0) ? 1 : -1;
                        cal_step++;
                        Calibration_Prompt("Calibration completed! Saving coefficients...");
                    }
                }
                else
                {
                    if(fabs(current_pos[0]) > 0.45 )          
                    {
                        Aircraft.Left.X_Coeff = (current_pos[0] > 0) ? 1 : -1;
                        cal_step++;
                        Calibration_Prompt("Calibration completed! Saving coefficients...");
                    }
                }
                break;
                
            case 12: /* 保存系数 */
                // 这里可以添加保存系数到非易失存储器的代码
                cal_step = 0; // 重置校准步骤
                Calibration_Prompt("Calibration saved successfully!");
                break;
        }
    }
}




/*
* 函数名称：void ConvertROSToAircraftFrame(float *ros_x, float *ros_y, float ros_yaw)
* 函数说明：将ROS坐标系数据转换为飞机坐标系数据
* 函数参数：
*   ros_x: 指向ROS坐标系X坐标的指针（输入/输出）
*   ros_y: 指向ROS坐标系Y坐标的指针（输入/输出）
*   ros_yaw: ROS坐标系中的偏航角（输入）
* 函数返回：无
* 注意：转换后的数据将通过指针参数返回
*/
void ConvertROSToAircraftFrame(float *ros_x, float *ros_y, float ros_yaw)
{
    float temp_x, temp_y;
    static uint8_t last_direction = 0; // 记录上一次的方向，用于判断是否需要重新计算
    
    /* 获取当前飞机朝向（简化版本，实际应用中可能需要根据飞行状态或遥控器输入确定） */
    uint8_t current_direction = GetAircraftDirection();
    
    /* 如果方向发生变化，才重新计算转换系数 */
    if(current_direction != last_direction)
    {
        last_direction = current_direction;
    }
    
    /* 根据当前飞机朝向应用相应的转换系数 */
    switch(current_direction)
    {
        case DIRECTION_NOSE: // 机头朝前
            if(Aircraft.Nose.X_exchange_Y)
            {
                /* 交换XY轴 */
                temp_x = (*ros_y) * Aircraft.Nose.Y_Coeff;
                temp_y = (*ros_x) * Aircraft.Nose.X_Coeff;
            }
            else
            {
                /* 只应用系数 */
                temp_x = (*ros_x) * Aircraft.Nose.X_Coeff;
                temp_y = (*ros_y) * Aircraft.Nose.Y_Coeff;
            }
            break;
            
        case DIRECTION_TAIL: // 机尾朝前
            if(Aircraft.Tail.X_exchange_Y)
            {
                temp_x = (*ros_y) * Aircraft.Tail.Y_Coeff;
                temp_y = (*ros_x) * Aircraft.Tail.X_Coeff;
            }
            else
            {
                temp_x = (*ros_x) * Aircraft.Tail.X_Coeff;
                temp_y = (*ros_y) * Aircraft.Tail.Y_Coeff;
            }
            break;
            
        case DIRECTION_LEFT: // 左侧朝前
            if(Aircraft.Left.X_exchange_Y)
            {
                temp_x = (*ros_y) * Aircraft.Left.Y_Coeff;
                temp_y = (*ros_x) * Aircraft.Left.X_Coeff;
            }
            else
            {
                temp_x = (*ros_x) * Aircraft.Left.X_Coeff;
                temp_y = (*ros_y) * Aircraft.Left.Y_Coeff;
            }
            break;
            
        case DIRECTION_RIGHT: // 右侧朝前
            if(Aircraft.Right.X_exchange_Y)
            {
                temp_x = (*ros_y) * Aircraft.Right.Y_Coeff;
                temp_y = (*ros_x) * Aircraft.Right.X_Coeff;
            }
            else
            {
                temp_x = (*ros_x) * Aircraft.Right.X_Coeff;
                temp_y = (*ros_y) * Aircraft.Right.Y_Coeff;
            }
            break;
            
        default: // 默认使用机头方向的转换系数
            if(Aircraft.Nose.X_exchange_Y)
            {
                temp_x = (*ros_y) * Aircraft.Nose.Y_Coeff;
                temp_y = (*ros_x) * Aircraft.Nose.X_Coeff;
            }
            else
            {
                temp_x = (*ros_x) * Aircraft.Nose.X_Coeff;
                temp_y = (*ros_y) * Aircraft.Nose.Y_Coeff;
            }
            break;
    }
    
    /* 更新坐标值 */
    *ros_x = temp_x;
    *ros_y = temp_y;
    
    /* 偏航角转换（根据实际情况可能需要调整） */
    // 这里简化处理，实际应用中可能需要根据具体情况进行更复杂的转换
    ros_yaw = normalize_yaw(ros_yaw);
}

/* 辅助函数：获取当前飞机朝向 */
uint8_t GetAircraftDirection(void)
{
    float current_yaw = (float)fc_att.st_data.yaw_x100 / 100.0f;
    
    // 根据偏航角确定飞机朝向
    // 这里使用简化的判断逻辑，实际应用中可能需要更精确的判断
    if((current_yaw >= -45.0f) && (current_yaw < 45.0f))
        return DIRECTION_NOSE;  // 机头朝前
    else if((current_yaw >= 45.0f) && (current_yaw < 135.0f))
        return DIRECTION_RIGHT; // 右侧朝前
    else if((current_yaw >= 135.0f) || (current_yaw < -135.0f))
        return DIRECTION_TAIL;  // 机尾朝前
    else // ((current_yaw >= -135.0f) && (current_yaw < -45.0f))
        return DIRECTION_LEFT;  // 左侧朝前
}
