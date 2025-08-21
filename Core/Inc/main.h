/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
#define LD1_Pin GPIO_PIN_13
#define LD1_GPIO_Port GPIOC
#define TDC1_X_Pin GPIO_PIN_0
#define TDC1_X_GPIO_Port GPIOA
#define TDC1_Y_Pin GPIO_PIN_1
#define TDC1_Y_GPIO_Port GPIOA
#define TDC2_X_Pin GPIO_PIN_2
#define TDC2_X_GPIO_Port GPIOA
#define TDC2_Y_Pin GPIO_PIN_3
#define TDC2_Y_GPIO_Port GPIOA
#define TDC_PUSH_Pin GPIO_PIN_4
#define TDC_PUSH_GPIO_Port GPIOA
#define TDC2_PUSH_Pin GPIO_PIN_5
#define TDC2_PUSH_GPIO_Port GPIOA
#define BRK_RET_Pin GPIO_PIN_0
#define BRK_RET_GPIO_Port GPIOB
#define BRK_EXT_Pin GPIO_PIN_1
#define BRK_EXT_GPIO_Port GPIOB
#define DOGFIGHT_Pin GPIO_PIN_10
#define DOGFIGHT_GPIO_Port GPIOB
#define MSSILE_Pin GPIO_PIN_11
#define MSSILE_GPIO_Port GPIOB
#define PINKY_AFT_Pin GPIO_PIN_12
#define PINKY_AFT_GPIO_Port GPIOB
#define PINKY_FWD_Pin GPIO_PIN_13
#define PINKY_FWD_GPIO_Port GPIOB
#define SLIDER_DOWN_Pin GPIO_PIN_3
#define SLIDER_DOWN_GPIO_Port GPIOB
#define SLIDER_HALF_DOWN_Pin GPIO_PIN_4
#define SLIDER_HALF_DOWN_GPIO_Port GPIOB
#define SLIDER_PUSH_Pin GPIO_PIN_5
#define SLIDER_PUSH_GPIO_Port GPIOB
#define SLIDER_HALF_UP_Pin GPIO_PIN_6
#define SLIDER_HALF_UP_GPIO_Port GPIOB
#define SLIDER_UP_Pin GPIO_PIN_7
#define SLIDER_UP_GPIO_Port GPIOB
#define TDC_CALBRATION_Pin GPIO_PIN_8
#define TDC_CALBRATION_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
