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
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim_ex.h"
#include "stm32f1xx_hal_uart.h"
#include  <string.h>
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
#define Cadence_LED_Pin GPIO_PIN_13
#define Cadence_LED_GPIO_Port GPIOC
#define Start_Stop_Button_Pin GPIO_PIN_0
#define Start_Stop_Button_GPIO_Port GPIOA
#define Cadence_setpoint_Pin GPIO_PIN_1
#define Cadence_setpoint_GPIO_Port GPIOA
#define Torque_setpoint_Pin GPIO_PIN_2
#define Torque_setpoint_GPIO_Port GPIOA
#define Speed_dir_Pin GPIO_PIN_3
#define Speed_dir_GPIO_Port GPIOA
#define Torque_Pin GPIO_PIN_8
#define Torque_GPIO_Port GPIOA
#define PAS2_Pin GPIO_PIN_9
#define PAS2_GPIO_Port GPIOA
#define PAS1_Pin GPIO_PIN_10
#define PAS1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define UserButton_Pin GPIO_PIN_0
#define UserButton_GPIO_Port GPIOA
#define Cadence_setpoint_Pin GPIO_PIN_1
#define Cadence_setpoint_GPIO_Port GPIOA
#define Torque_setpoint_Pin GPIO_PIN_2
#define Torque_setpoint_GPIO_Port GPIOA
#define Torquesignal_Pin GPIO_PIN_8
#define Torquesignal_GPIO_Port GPIOA
#define PAS_signal_Pin GPIO_PIN_3
#define PAS_signal_GPIO_Port GPIOA
#define Q_PAS1_Pin GPIO_PIN_10
#define Q_PAS1_GPIO_Port GPIOA
#define Q_PAS2_Pin GPIO_PIN_9
#define Q_PAS2_GPIO_Port GPIOA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
