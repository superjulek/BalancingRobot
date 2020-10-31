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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define R_DIR_Pin GPIO_PIN_0
#define R_DIR_GPIO_Port GPIOC
#define L_ENABLE_Pin GPIO_PIN_1
#define L_ENABLE_GPIO_Port GPIOC
#define R_ENABLE_Pin GPIO_PIN_2
#define R_ENABLE_GPIO_Port GPIOC
#define BAT_VOL_Pin GPIO_PIN_3
#define BAT_VOL_GPIO_Port GPIOC
#define L_MS3_Pin GPIO_PIN_1
#define L_MS3_GPIO_Port GPIOA
#define L_MS2_Pin GPIO_PIN_4
#define L_MS2_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define L_MS1_Pin GPIO_PIN_0
#define L_MS1_GPIO_Port GPIOB
#define INT_Pin GPIO_PIN_8
#define INT_GPIO_Port GPIOC
#define MPU_POWER_Pin GPIO_PIN_9
#define MPU_POWER_GPIO_Port GPIOC
#define L_DIR_Pin GPIO_PIN_7
#define L_DIR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
