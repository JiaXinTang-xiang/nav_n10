/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp.h"
#include "Chassis_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool init_finished = 0;
float ypr[3];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_TIM6_Init();
  MX_SPI2_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_USART6_UART_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  TB6612_Init();
  delay_init();
  IMU_init();
  OLED_init();
  Servo_Init();
  Relay_Init();
  SPI_IMU_CS(1);
  delay_ms(100);
  UART_Init(&huart2, UART_Host_Call_Back, UART_BUFFER_SIZE);   /* USART2 = 底盘协议 (Jetson) */
  UART_Init(&huart4, UART_WireLess_Call_Back, UART_BUFFER_SIZE);
  RGB_Control(0,0,0);
  IMU_Ctrl_Init();
  Chassis_Init();          /* 编码器(TIM1/TIM2 输入捕获) + 速度环 PID */
  init_finished = 1;
  
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	
	  IMU_getYawPitchRoll(ypr);
//	  debug_printf("\tangle:%.2f\t%.2f\t%.2f\r\n", ypr[0], ypr[1], ypr[2]);

	  /* ============ 底盘调试: 按 USERKEY 循环切换八个模式 ============
	     模式 0  标定显示 : 手推车看计数/轮速/距离/里程计
	     模式 1  开环基准 : Load(-20,-20) 那个 PWM, 看 act 是多少
	     模式 2  速度环   : 两轮闭环跑 0.2 m/s, 看 act 跟不跟得住 tgt
	     模式 3  低速陀螺仪直行: 等效 Load(-20,-20)，但启用闭环纠偏
	     模式 4  编码器自检: 期望约 400,400（200 次翻转的边沿总数）
	     模式 5  陀螺仪数据显示: gz 和修正量
	     模式 6  角度锁定直行: 锁定启动 yaw，显示 T/N、E/C、act、pwm
	     模式 7  编码器边沿诊断: 不动车，显示 ISR Hz / cnt / distance / odom
	     模式 1/2/3/6 会动车；切走自动停车，直行效果在平地低速验证。
	     ============================================================== */
	  {
		  static uint8_t mode = 0;
		  uint8_t key_num = Key_GetNum();

		  /* 电机测试运行时长按：立即停车，回模式 0，并保留最终计数。
		     其余页面长按：进入模式 7 并清零，供手推/编码器测试。 */
		  if (key_num == USERKEY_LONG) {
			  if ((mode == 1u) || (mode == 2u) || (mode == 3u) || (mode == 6u)) {
				  Chassis_TestStop();
				  mode = 0u;
			  } else {
				  mode = 7u;
				  Chassis_DebugResetIsr();
			  }
		  } else if (key_num == USERKEY_SHORT) {
			  /* 离开任何会转的模式前, 先停车 */
			  if ((mode == 1u) || (mode == 2u) || (mode == 3u) || (mode == 6u)) {
				  Chassis_TestStop();
			  }

			  mode = (uint8_t)((mode + 1u) % 8u);

			  if (mode == 1u) {
				  Chassis_TestOpenLoopStart();   /* 开环 PWM=20 基准 */
			  } else if (mode == 2u) {
				  Chassis_TestSpeedStart();      /* 闭环 0.2 m/s */
			  } else if (mode == 3u) {
				  Chassis_TestGyroStraightStart();
			  } else if (mode == 4u) {
				  Chassis_SelfTest();            /* 编码器通路自检 */
			  } else if (mode == 6u) {
				  Chassis_TestAngleStraightStart();
			  } else if (mode == 7u) {
				  Chassis_DebugResetIsr();
			  }
		  }

		  switch (mode) {
		  case 1u:
		  case 2u:
		  case 3u:
			  Chassis_DebugDisplaySpeedTest();
			  break;
		  case 4u:
			  Chassis_DebugDisplaySelfTest();
			  break;
		  case 5u:
			  Chassis_DebugDisplayGyro();
			  break;
		  case 6u:
			  Chassis_DebugDisplayAngleTest();
			  break;
		  case 7u:
			  Chassis_DebugDisplayIsr();
			  break;
		  default:
			  if (Chassis_IsAngleHoldEnabled() != 0u) {
				  Chassis_DebugDisplayAngleTest(); /* 正式直行显示 AH T/N */
			  } else {
				  Chassis_DebugDisplay();
			  }
			  break;
		  }
	  }

    /* USER CODE END WHILE */
 
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
