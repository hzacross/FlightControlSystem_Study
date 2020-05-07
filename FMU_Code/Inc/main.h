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
#define GYRO_CS_Pin GPIO_PIN_13
#define GYRO_CS_GPIO_Port GPIOC
#define MPU_CS_Pin GPIO_PIN_2
#define MPU_CS_GPIO_Port GPIOC
#define FMU_UART7_RX_Pin GPIO_PIN_7
#define FMU_UART7_RX_GPIO_Port GPIOE
#define FMU_UART7_TX_Pin GPIO_PIN_8
#define FMU_UART7_TX_GPIO_Port GPIOE
#define FMU_LED_AMBER_Pin GPIO_PIN_12
#define FMU_LED_AMBER_GPIO_Port GPIOE
#define SERIAL_FMU_TO_IO_Pin GPIO_PIN_6
#define SERIAL_FMU_TO_IO_GPIO_Port GPIOC
#define SERIAL_IO_TO_FMU_Pin GPIO_PIN_7
#define SERIAL_IO_TO_FMU_GPIO_Port GPIOC
#define datalink_TX_Pin GPIO_PIN_5
#define datalink_TX_GPIO_Port GPIOD
#define datalink_RX_Pin GPIO_PIN_6
#define datalink_RX_GPIO_Port GPIOD
#define BARO_CS_Pin GPIO_PIN_7
#define BARO_CS_GPIO_Port GPIOD
#define FMU_I2C1_SCL_Pin GPIO_PIN_8
#define FMU_I2C1_SCL_GPIO_Port GPIOB
#define FMU_I2C1_SDA_Pin GPIO_PIN_9
#define FMU_I2C1_SDA_GPIO_Port GPIOB
#define FMU_UART8_RX_Pin GPIO_PIN_0
#define FMU_UART8_RX_GPIO_Port GPIOE
#define FMU_UART_TX_Pin GPIO_PIN_1
#define FMU_UART_TX_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
