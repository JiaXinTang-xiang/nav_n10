/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY2_Pin GPIO_PIN_2
#define KEY2_GPIO_Port GPIOE
#define IO1_Pin GPIO_PIN_5
#define IO1_GPIO_Port GPIOE
#define IO2_Pin GPIO_PIN_6
#define IO2_GPIO_Port GPIOE
#define USER_KEY_Pin GPIO_PIN_13
#define USER_KEY_GPIO_Port GPIOC
#define USER_KEY_EXTI_IRQn EXTI15_10_IRQn
#define L_Pin GPIO_PIN_0
#define L_GPIO_Port GPIOC
#define ML_Pin GPIO_PIN_1
#define ML_GPIO_Port GPIOC
#define MR_Pin GPIO_PIN_2
#define MR_GPIO_Port GPIOC
#define R_Pin GPIO_PIN_3
#define R_GPIO_Port GPIOC
#define IO4_Pin GPIO_PIN_4
#define IO4_GPIO_Port GPIOA
#define LLL_Pin GPIO_PIN_5
#define LLL_GPIO_Port GPIOA
#define LL_Pin GPIO_PIN_6
#define LL_GPIO_Port GPIOA
#define RR_Pin GPIO_PIN_4
#define RR_GPIO_Port GPIOC
#define RRR_Pin GPIO_PIN_5
#define RRR_GPIO_Port GPIOC
#define RED_Pin GPIO_PIN_0
#define RED_GPIO_Port GPIOB
#define PWMB_Pin GPIO_PIN_1
#define PWMB_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOB
#define KEY3_Pin GPIO_PIN_7
#define KEY3_GPIO_Port GPIOE
#define GREEN_Pin GPIO_PIN_8
#define GREEN_GPIO_Port GPIOE
#define BLUE_Pin GPIO_PIN_10
#define BLUE_GPIO_Port GPIOE
#define AIN1_Pin GPIO_PIN_14
#define AIN1_GPIO_Port GPIOE
#define AIN2_Pin GPIO_PIN_15
#define AIN2_GPIO_Port GPIOE
#define OLED_SCL_Pin GPIO_PIN_10
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_11
#define OLED_SDA_GPIO_Port GPIOB
#define SPI_IMU_CS_Pin GPIO_PIN_12
#define SPI_IMU_CS_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_8
#define BIN1_GPIO_Port GPIOD
#define BIIN2_Pin GPIO_PIN_9
#define BIIN2_GPIO_Port GPIOD
#define BUZZER_Pin GPIO_PIN_10
#define BUZZER_GPIO_Port GPIOD
#define SG901_Pin GPIO_PIN_12
#define SG901_GPIO_Port GPIOD
#define SG902_Pin GPIO_PIN_13
#define SG902_GPIO_Port GPIOD
#define SG903_Pin GPIO_PIN_14
#define SG903_GPIO_Port GPIOD
#define SG904_Pin GPIO_PIN_15
#define SG904_GPIO_Port GPIOD
#define RGB_Pin GPIO_PIN_9
#define RGB_GPIO_Port GPIOC
#define SPI_FLASH_CS_Pin GPIO_PIN_15
#define SPI_FLASH_CS_GPIO_Port GPIOA
#define OLED_D0_Pin GPIO_PIN_10
#define OLED_D0_GPIO_Port GPIOC
#define OLED_DC_Pin GPIO_PIN_11
#define OLED_DC_GPIO_Port GPIOC
#define OLED_D1_Pin GPIO_PIN_12
#define OLED_D1_GPIO_Port GPIOC
#define OLED_RES_Pin GPIO_PIN_0
#define OLED_RES_GPIO_Port GPIOD
#define OLED_CS_Pin GPIO_PIN_1
#define OLED_CS_GPIO_Port GPIOD
#define PWMA_Pin GPIO_PIN_4
#define PWMA_GPIO_Port GPIOB
#define IO3_Pin GPIO_PIN_7
#define IO3_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
