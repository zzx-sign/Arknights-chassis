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
#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOE
#define KEY0_Pin GPIO_PIN_4
#define KEY0_GPIO_Port GPIOE
#define LED_2_Pin GPIO_PIN_5
#define LED_2_GPIO_Port GPIOE
#define Driver1AIN1_Pin GPIO_PIN_0
#define Driver1AIN1_GPIO_Port GPIOF
#define Driver1AIN2_Pin GPIO_PIN_1
#define Driver1AIN2_GPIO_Port GPIOF
#define Driver2AIN1_Pin GPIO_PIN_2
#define Driver2AIN1_GPIO_Port GPIOF
#define Driver2AIN2_Pin GPIO_PIN_3
#define Driver2AIN2_GPIO_Port GPIOF
#define Driver3AIN1_Pin GPIO_PIN_4
#define Driver3AIN1_GPIO_Port GPIOF
#define Driver3AIN2_Pin GPIO_PIN_5
#define Driver3AIN2_GPIO_Port GPIOF
#define Driver4AIN1_Pin GPIO_PIN_6
#define Driver4AIN1_GPIO_Port GPIOF
#define Driver4AIN2_Pin GPIO_PIN_7
#define Driver4AIN2_GPIO_Port GPIOF
#define STBY1_Pin GPIO_PIN_4
#define STBY1_GPIO_Port GPIOA
#define STBY2_Pin GPIO_PIN_5
#define STBY2_GPIO_Port GPIOA
#define Encoder2_1_Pin GPIO_PIN_6
#define Encoder2_1_GPIO_Port GPIOA
#define Encoder2_2_Pin GPIO_PIN_7
#define Encoder2_2_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOE
#define PWM2_Pin GPIO_PIN_11
#define PWM2_GPIO_Port GPIOE
#define PWM3_Pin GPIO_PIN_13
#define PWM3_GPIO_Port GPIOE
#define PWM4_Pin GPIO_PIN_14
#define PWM4_GPIO_Port GPIOE
#define Camera_TX_Pin GPIO_PIN_10
#define Camera_TX_GPIO_Port GPIOB
#define Camera_RX_Pin GPIO_PIN_11
#define Camera_RX_GPIO_Port GPIOB
#define Encoder3_2_Pin GPIO_PIN_12
#define Encoder3_2_GPIO_Port GPIOD
#define Encoder3_1_Pin GPIO_PIN_13
#define Encoder3_1_GPIO_Port GPIOD
#define Encoder4_2_Pin GPIO_PIN_6
#define Encoder4_2_GPIO_Port GPIOC
#define Encoder4_1_Pin GPIO_PIN_7
#define Encoder4_1_GPIO_Port GPIOC
#define Debug_TX_Pin GPIO_PIN_9
#define Debug_TX_GPIO_Port GPIOA
#define Debug_RX_Pin GPIO_PIN_10
#define Debug_RX_GPIO_Port GPIOA
#define Encoder1_2_Pin GPIO_PIN_15
#define Encoder1_2_GPIO_Port GPIOA
#define Up_Communication_TX_Pin GPIO_PIN_10
#define Up_Communication_TX_GPIO_Port GPIOC
#define Up_Communication_RX_Pin GPIO_PIN_11
#define Up_Communication_RX_GPIO_Port GPIOC
#define Bluetooth_TX_Pin GPIO_PIN_5
#define Bluetooth_TX_GPIO_Port GPIOD
#define Bluetooth_RX_Pin GPIO_PIN_6
#define Bluetooth_RX_GPIO_Port GPIOD
#define Encoder1_1_Pin GPIO_PIN_3
#define Encoder1_1_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_5
#define LED_1_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
