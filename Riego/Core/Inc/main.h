/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define LED_AUTOM_Pin GPIO_PIN_2
#define LED_AUTOM_GPIO_Port GPIOE
#define LED_MANUAL_Pin GPIO_PIN_4
#define LED_MANUAL_GPIO_Port GPIOE
#define MOTOR_PWM_Pin GPIO_PIN_6
#define MOTOR_PWM_GPIO_Port GPIOE
#define DATA_DS18B20_Pin GPIO_PIN_11
#define DATA_DS18B20_GPIO_Port GPIOB
#define MOTOR_INI1_Pin GPIO_PIN_7
#define MOTOR_INI1_GPIO_Port GPIOC
#define MOTOR_INI2_Pin GPIO_PIN_9
#define MOTOR_INI2_GPIO_Port GPIOC
#define PULSADOR_MODE_Pin GPIO_PIN_1
#define PULSADOR_MODE_GPIO_Port GPIOD
#define PULSADOR_MODE_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
