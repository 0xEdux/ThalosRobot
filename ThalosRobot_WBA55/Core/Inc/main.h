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
#include "stm32wbaxx_hal.h"

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
#define MOTORA_IN2_Pin GPIO_PIN_6
#define MOTORA_IN2_GPIO_Port GPIOA
#define MOTORA_IN1_Pin GPIO_PIN_5
#define MOTORA_IN1_GPIO_Port GPIOA
#define MOTORB_IN2_Pin GPIO_PIN_13
#define MOTORB_IN2_GPIO_Port GPIOC
#define MOTORB_IN1_Pin GPIO_PIN_7
#define MOTORB_IN1_GPIO_Port GPIOB
#define ENC_2_Pin GPIO_PIN_6
#define ENC_2_GPIO_Port GPIOB
#define ENC_2_EXTI_IRQn EXTI6_IRQn
#define ENC_1_Pin GPIO_PIN_5
#define ENC_1_GPIO_Port GPIOB
#define ENC_1_EXTI_IRQn EXTI5_IRQn
#define TCRT_CH1_Pin GPIO_PIN_11
#define TCRT_CH1_GPIO_Port GPIOA
#define VL_RIGHT_Pin GPIO_PIN_2
#define VL_RIGHT_GPIO_Port GPIOB
#define VL_RIGHT_EXTI_IRQn EXTI2_IRQn
#define VL_LEFT_Pin GPIO_PIN_1
#define VL_LEFT_GPIO_Port GPIOB
#define VL_LEFT_EXTI_IRQn EXTI1_IRQn
#define VL_FRONT_Pin GPIO_PIN_0
#define VL_FRONT_GPIO_Port GPIOB
#define VL_FRONT_EXTI_IRQn EXTI0_IRQn
#define TCRT_CH2_Pin GPIO_PIN_15
#define TCRT_CH2_GPIO_Port GPIOB
#define TCRT_CH3_Pin GPIO_PIN_3
#define TCRT_CH3_GPIO_Port GPIOH
#define TCRT_CH4_Pin GPIO_PIN_9
#define TCRT_CH4_GPIO_Port GPIOA
#define TCRT_CH5_Pin GPIO_PIN_14
#define TCRT_CH5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
