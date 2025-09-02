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
#include "stm32g0xx_hal.h"

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
#define VOL_P_Pin GPIO_PIN_0
#define VOL_P_GPIO_Port GPIOB
#define CHAN_M_Pin GPIO_PIN_1
#define CHAN_M_GPIO_Port GPIOB
#define CHAN_P_Pin GPIO_PIN_2
#define CHAN_P_GPIO_Port GPIOB
#define DIG1_Pin GPIO_PIN_11
#define DIG1_GPIO_Port GPIOB
#define DIG2_Pin GPIO_PIN_12
#define DIG2_GPIO_Port GPIOB
#define DIG4_Pin GPIO_PIN_13
#define DIG4_GPIO_Port GPIOB
#define DIG8_Pin GPIO_PIN_14
#define DIG8_GPIO_Port GPIOB
#define IND_Pin GPIO_PIN_15
#define IND_GPIO_Port GPIOB
#define VOL_M_Pin GPIO_PIN_3
#define VOL_M_GPIO_Port GPIOB
#define ADDR_SEL_Pin GPIO_PIN_5
#define ADDR_SEL_GPIO_Port GPIOB
#define GPO1_Pin GPIO_PIN_8
#define GPO1_GPIO_Port GPIOB
#define GPO2_Pin GPIO_PIN_9
#define GPO2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
