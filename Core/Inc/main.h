/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AIN1_Pin GPIO_PIN_0
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_1
#define AIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_2
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_10
#define BIN2_GPIO_Port GPIOB
#define LED_RED2_Pin GPIO_PIN_12
#define LED_RED2_GPIO_Port GPIOB
#define LED_BLUE2_Pin GPIO_PIN_13
#define LED_BLUE2_GPIO_Port GPIOB
#define LED_RED1_Pin GPIO_PIN_14
#define LED_RED1_GPIO_Port GPIOB
#define LED_BLUE1_Pin GPIO_PIN_15
#define LED_BLUE1_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_11
#define SW1_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_12
#define SW2_GPIO_Port GPIOA
#define STBY_Pin GPIO_PIN_15
#define STBY_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_8
#define PWMB_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_9
#define PWMA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
