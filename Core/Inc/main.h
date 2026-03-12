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
#define RS485_REDE_Pin GPIO_PIN_6
#define RS485_REDE_GPIO_Port GPIOF
#define LED_Pin GPIO_PIN_9
#define LED_GPIO_Port GPIOF
#define AD_RESET_Pin GPIO_PIN_7
#define AD_RESET_GPIO_Port GPIOA
#define AD_OS1_Pin GPIO_PIN_4
#define AD_OS1_GPIO_Port GPIOC
#define AD_OS2_Pin GPIO_PIN_5
#define AD_OS2_GPIO_Port GPIOC
#define AD_CONVST_Pin GPIO_PIN_0
#define AD_CONVST_GPIO_Port GPIOB
#define AD_RANGE_Pin GPIO_PIN_1
#define AD_RANGE_GPIO_Port GPIOB
#define AD_OS0_Pin GPIO_PIN_2
#define AD_OS0_GPIO_Port GPIOB
#define AD_CS_Pin GPIO_PIN_11
#define AD_CS_GPIO_Port GPIOF
#define DAC8568_LDAC_Pin GPIO_PIN_10
#define DAC8568_LDAC_GPIO_Port GPIOD
#define DAC8568_SYNC_Pin GPIO_PIN_12
#define DAC8568_SYNC_GPIO_Port GPIOD
#define DAC8568_DIN_Pin GPIO_PIN_14
#define DAC8568_DIN_GPIO_Port GPIOD
#define DAC8568_CLK_Pin GPIO_PIN_2
#define DAC8568_CLK_GPIO_Port GPIOG
#define DAC8568_CLR_Pin GPIO_PIN_4
#define DAC8568_CLR_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
