/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Encoder_2A_Pin GPIO_PIN_0
#define Encoder_2A_GPIO_Port GPIOA
#define Encoder_2B_Pin GPIO_PIN_1
#define Encoder_2B_GPIO_Port GPIOA
#define Encoder_1A_Pin GPIO_PIN_5
#define Encoder_1A_GPIO_Port GPIOA
#define Motor_1A_Pin GPIO_PIN_6
#define Motor_1A_GPIO_Port GPIOA
#define Motor_1B_Pin GPIO_PIN_7
#define Motor_1B_GPIO_Port GPIOA
#define GPS_Rx_Pin GPIO_PIN_6
#define GPS_Rx_GPIO_Port GPIOC
#define GPS_Tx_Pin GPIO_PIN_7
#define GPS_Tx_GPIO_Port GPIOC
#define Encoder_1B_Pin GPIO_PIN_3
#define Encoder_1B_GPIO_Port GPIOB
#define Motor_2A_Pin GPIO_PIN_6
#define Motor_2A_GPIO_Port GPIOB
#define Motor_2B_Pin GPIO_PIN_7
#define Motor_2B_GPIO_Port GPIOB
#define IMU_I2C_SCL_Pin GPIO_PIN_8
#define IMU_I2C_SCL_GPIO_Port GPIOB
#define IMU_I2C_SDA_Pin GPIO_PIN_9
#define IMU_I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
