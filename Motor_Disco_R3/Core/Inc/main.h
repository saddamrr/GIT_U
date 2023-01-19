/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define enc4_b_Pin GPIO_PIN_5
#define enc4_b_GPIO_Port GPIOE
#define enc4_b_EXTI_IRQn EXTI9_5_IRQn
#define enc4_a_Pin GPIO_PIN_6
#define enc4_a_GPIO_Port GPIOE
#define enc4_a_EXTI_IRQn EXTI9_5_IRQn
#define enc2_b_Pin GPIO_PIN_12
#define enc2_b_GPIO_Port GPIOE
#define enc2_b_EXTI_IRQn EXTI15_10_IRQn
#define enc1_a_Pin GPIO_PIN_8
#define enc1_a_GPIO_Port GPIOD
#define enc1_a_EXTI_IRQn EXTI9_5_IRQn
#define enc1_b_Pin GPIO_PIN_9
#define enc1_b_GPIO_Port GPIOD
#define enc1_b_EXTI_IRQn EXTI9_5_IRQn
#define enc3_a_Pin GPIO_PIN_7
#define enc3_a_GPIO_Port GPIOD
#define enc3_a_EXTI_IRQn EXTI9_5_IRQn
#define enc2_a_Pin GPIO_PIN_3
#define enc2_a_GPIO_Port GPIOB
#define enc2_a_EXTI_IRQn EXTI3_IRQn
#define enc3_b_Pin GPIO_PIN_4
#define enc3_b_GPIO_Port GPIOB
#define enc3_b_EXTI_IRQn EXTI4_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
