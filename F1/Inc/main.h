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
#define IO_CH1_Pin GPIO_PIN_0
#define IO_CH1_GPIO_Port GPIOA
#define IO_CH2_Pin GPIO_PIN_1
#define IO_CH2_GPIO_Port GPIOA
#define SERIAL_TO_FMU_TX_Pin GPIO_PIN_2
#define SERIAL_TO_FMU_TX_GPIO_Port GPIOA
#define SERIAL_TO_FMU_RX_Pin GPIO_PIN_3
#define SERIAL_TO_FMU_RX_GPIO_Port GPIOA
#define IO_CH5_Pin GPIO_PIN_6
#define IO_CH5_GPIO_Port GPIOA
#define IO_CH6_Pin GPIO_PIN_7
#define IO_CH6_GPIO_Port GPIOA
#define IO_CH7_Pin GPIO_PIN_0
#define IO_CH7_GPIO_Port GPIOB
#define IO_CH8_Pin GPIO_PIN_1
#define IO_CH8_GPIO_Port GPIOB
#define SUBS_OUT_Pin GPIO_PIN_10
#define SUBS_OUT_GPIO_Port GPIOB
#define SUBS_INPUT_Pin GPIO_PIN_11
#define SUBS_INPUT_GPIO_Port GPIOB
#define IO_LED_SAFETY_Pin GPIO_PIN_13
#define IO_LED_SAFETY_GPIO_Port GPIOB
#define IO_LED_BLUE_Pin GPIO_PIN_14
#define IO_LED_BLUE_GPIO_Port GPIOB
#define IO_LED_AMBER_Pin GPIO_PIN_15
#define IO_LED_AMBER_GPIO_Port GPIOB
#define PPM_INPUT_Pin GPIO_PIN_8
#define PPM_INPUT_GPIO_Port GPIOA
#define SAFETY_Pin GPIO_PIN_5
#define SAFETY_GPIO_Port GPIOB
#define IO_CH3_Pin GPIO_PIN_8
#define IO_CH3_GPIO_Port GPIOB
#define IO_CH4_Pin GPIO_PIN_9
#define IO_CH4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
