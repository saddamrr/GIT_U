/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define BZR_Pin GPIO_PIN_3
#define BZR_GPIO_Port GPIOE
#define BTN5_Pin GPIO_PIN_4
#define BTN5_GPIO_Port GPIOE
#define RS_Pin GPIO_PIN_5
#define RS_GPIO_Port GPIOE
#define BTN4_Pin GPIO_PIN_6
#define BTN4_GPIO_Port GPIOE
#define D4_Pin GPIO_PIN_3
#define D4_GPIO_Port GPIOA
#define BTN3_Pin GPIO_PIN_5
#define BTN3_GPIO_Port GPIOA
#define BTN2_Pin GPIO_PIN_7
#define BTN2_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_5
#define BTN1_GPIO_Port GPIOC
#define BTN6_Pin GPIO_PIN_14
#define BTN6_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_8
#define D5_GPIO_Port GPIOD
#define D6_Pin GPIO_PIN_9
#define D6_GPIO_Port GPIOD
#define EN_Pin GPIO_PIN_10
#define EN_GPIO_Port GPIOD
#define D7_Pin GPIO_PIN_7
#define D7_GPIO_Port GPIOC
#define ADATA_Pin GPIO_PIN_7
#define ADATA_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
