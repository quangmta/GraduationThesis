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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BLM_PWM_U_Pin GPIO_PIN_0
#define BLM_PWM_U_GPIO_Port GPIOA
#define BLM_PWM_I_Pin GPIO_PIN_1
#define BLM_PWM_I_GPIO_Port GPIOA
#define VLV1_Pin GPIO_PIN_2
#define VLV1_GPIO_Port GPIOA
#define BLM_P0A_Pin GPIO_PIN_6
#define BLM_P0A_GPIO_Port GPIOA
#define BLM_P0B_Pin GPIO_PIN_7
#define BLM_P0B_GPIO_Port GPIOA
#define BLM_P0C_Pin GPIO_PIN_0
#define BLM_P0C_GPIO_Port GPIOB
#define SW0_Pin GPIO_PIN_10
#define SW0_GPIO_Port GPIOB
#define SW0_EXTI_IRQn EXTI15_10_IRQn
#define SW1_Pin GPIO_PIN_11
#define SW1_GPIO_Port GPIOB
#define SW1_EXTI_IRQn EXTI15_10_IRQn
#define SPI_SS_Pin GPIO_PIN_14
#define SPI_SS_GPIO_Port GPIOB
#define SPI_MISO_Pin GPIO_PIN_15
#define SPI_MISO_GPIO_Port GPIOB
#define SPI_SCK_Pin GPIO_PIN_8
#define SPI_SCK_GPIO_Port GPIOA
#define BLM_E_Pin GPIO_PIN_3
#define BLM_E_GPIO_Port GPIOB
#define BLM_ER_Pin GPIO_PIN_4
#define BLM_ER_GPIO_Port GPIOB
#define BLM_ER_EXTI_IRQn EXTI4_IRQn
#define BLM_SI_Pin GPIO_PIN_5
#define BLM_SI_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define PRESSURE_MAX 3300
#define PRESSURE_MIN 1100
#define PRESSURE_NORMAL 1298.7
#define FREQUENCY_MAX 1600
#define FREQUENCY_MIN 80
#define PWM_MAX 399
#define PWM_MIN 0
#define PI 3.1415926
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
