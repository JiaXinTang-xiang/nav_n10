#include "time5_8.h"

/*
	函数：TIM5_Init(u16 arr,u16 psc)
	此函数用于初始化TIM5和PWM输出
	arr = 200 : psc = 8400
*/
void TIM5_Init(u16 arr,u16 psc)                             
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	      //  使能GPIOA、TIM5的时钟
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM5);         //  将GPIOA2-3引脚复用给TIM5
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM5);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;                   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                  //  复用模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                //  复用推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                  
	GPIO_Init(GPIOA,&GPIO_InitStructure);                         //  初始化1GPIOA  
	  
	TIM_TimeBaseStructure.TIM_Prescaler = (psc-1);                //   设置预分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;     //   向上计数
	TIM_TimeBaseStructure.TIM_Period = (arr-1);                   //   设置装载值        
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;         //   时钟一分频
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);                // TIM5参数初始化
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;              // 通道模式：PWM1  
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;       // 通道极性低	
	TIM_OCInitStructure.TIM_Pulse = 0; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  // 输出使能          
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);                       // TIM5通道3初始化 
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable); 
	
	TIM_OCInitStructure.TIM_Pulse = 0; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  // 输出使能
	TIM_OC4Init(TIM5,&TIM_OCInitStructure);						   // TIM5通道4初始化      
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
    TIM_ARRPreloadConfig(TIM5,ENABLE); 
	TIM_Cmd(TIM5, ENABLE);  	                                   //  TIM5使能								  
}  

/*
	函数：Tim5_Servo360_On_Off(u8 mode,u8 pwm_chx)
	此函数用于控制360°舵机
	mode(0:停,1:放，2:收)
	pwm_chx(PWM_CH6:pwm通道6，PWM_CH5:pwm通道5)
*/
void Tim5_Servo360_On_Off(u8 mode,u8 pwm_chx)   
{
	if(pwm_chx == PWM_CH6)
	{
		switch(mode)
		{
			case 0:{TIM_SetCompare3(TIM5,185);}break;//pwm6xd   停
			case 1:{TIM_SetCompare3(TIM5,180);}break;//pwm6xd   放
			case 2:{TIM_SetCompare3(TIM5,190);}break;//pwm6xd   收
			default:
					break;
		}
	}
	else if(pwm_chx == PWM_CH5)
	{
		switch(mode)
		{
			case 0:{TIM_SetCompare4(TIM5,185);}break;//pwm5xd   停
			case 1:{TIM_SetCompare4(TIM5,180);}break;//pwm5xd   放
			case 2:{TIM_SetCompare4(TIM5,190);}break;//pwm5xd   收
			default:
					break;
		}
	}
}
/*
	函数：Tim5_Servo180_On_Off(u8 mode,u8 pwm_chx)
	此函数用于控制180°舵机
	Angle:0°，45°，90°，135°，180°
	pwm_chx(PWM_CH6:pwm通道6，PWM_CH5:pwm通道5)
*/
void Tim5_Servo180_On_Off(u8 Angle,u8 pwm_chx)   
{
	u8 ret = 0;
	ret = 185 + (s16)((90 - Angle)*1/ 9); 
	switch(pwm_chx)
	{
		case PWM_CH6:TIM_SetCompare3(TIM5,ret);break;
		case PWM_CH5:TIM_SetCompare4(TIM5,ret);break;
		default:
			break;
	}
}

/*
	TIM8_Init(u16 arr,u16 psc)
	此函数用于初始化TIM5和PWM输出
	arr = 200 : psc = 16800
*/
void TIM8_Init(u16 arr,u16 psc)  
{
	GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);                //  使能GPIO和TIM8的时钟
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; 
    GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AF;                        //  复用模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;              
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                     //  推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                       //  带有上拉电阻
    GPIO_Init(GPIOC,&GPIO_InitStructure);                              //  GPIO初始化
    
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8);              //  将PC8复用给TIM8
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8);              //  将PC8复用给TIM8
	
    TIM_TimeBaseInitStructure.TIM_Period = (arr-1);                        //  自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = (psc-1);                     //  预分频值
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;    //  向上计数
    TIM_TimeBaseInitStructure.TIM_ClockDivision =TIM_CKD_DIV1;         //  时钟因子
    TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStructure);                 //  初始化定时器8
    
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	
	TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC3Init(TIM8,&TIM_OCInitStructure);                            //  通道三初始化
    TIM_OC3PreloadConfig(TIM8,TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC4Init(TIM8,&TIM_OCInitStructure);                            //  通道四初始化
    TIM_OC4PreloadConfig(TIM8,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM8,ENABLE); 
    TIM_CtrlPWMOutputs(TIM8,ENABLE);                                   //  高级定时器独有，PWM输出控制
    TIM_Cmd(TIM8,ENABLE);                                              //  定时器使能                                            //  定时器使能
}

/*
	函数：Tim8_Servo180_On_Off(u8 mode,u8 pwm_chx)
	此函数用于控制180°舵机
	Angle: 0°，45°，90°，135°，180°
	pwm_chx(PWM_CH6:pwm通道8，PWM_CH5:pwm通道7 )
*/
void Tim8_Servo180_On_Off(u8 Angle,u8 pwm_chx)   
{
	u8 ret = 0;
	ret = 185 + (s16)((90 - Angle)*1/ 9); 
	switch(pwm_chx)
	{
		case PWM_CH8:TIM_SetCompare3(TIM8,ret);break;
		case PWM_CH7:TIM_SetCompare4(TIM8,ret);break;
		default:
			break;
	}
}

/*
	函数：Tim8_Servo360_On_Off(u8 mode,u8 pwm_chx)
	此函数用于控制360°舵机
	mode(0:停,1:放，2:收)
	pwm_chx(PWM_CH6:pwm通道6，PWM_CH7:pwm通道7)
*/
void Tim8_Servo360_On_Off(u8 mode,u8 pwm_chx)   
{
	if(pwm_chx == PWM_CH8)
	{
		switch(mode)
		{
			case 0:{TIM_SetCompare3(TIM8,185);}break;//pwm6xd   停
			case 1:{TIM_SetCompare3(TIM8,180);}break;//pwm6xd   放
			case 2:{TIM_SetCompare3(TIM8,190);}break;//pwm6xd   收
			default:
					break;
		}
	}
	else if(pwm_chx == PWM_CH7)
	{
		switch(mode)
		{
			case 0:{TIM_SetCompare4(TIM8,185);}break;//pwm5xd   停
			case 1:{TIM_SetCompare4(TIM8,180);}break;//pwm5xd   放
			case 2:{TIM_SetCompare4(TIM8,190);}break;//pwm5xd   收
			default:
					break;
		}
	}
}


