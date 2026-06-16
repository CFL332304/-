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
#define CSN_Pin GPIO_PIN_0
#define CSN_GPIO_Port GPIOA
#define CE_Pin GPIO_PIN_1
#define CE_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_2
#define SCK_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_3
#define MOSI_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_4
#define MISO_GPIO_Port GPIOA
#define IRQ_Pin GPIO_PIN_5
#define IRQ_GPIO_Port GPIOA
#define LH_Pin GPIO_PIN_6
#define LH_GPIO_Port GPIOA
#define LV_Pin GPIO_PIN_7
#define LV_GPIO_Port GPIOA
#define RH_Pin GPIO_PIN_0
#define RH_GPIO_Port GPIOB
#define RV_Pin GPIO_PIN_1
#define RV_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_10
#define SW1_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_11
#define SW2_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_14
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_15
#define OLED_SDA_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_10
#define SW3_GPIO_Port GPIOA
#define SW4_Pin GPIO_PIN_11
#define SW4_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
