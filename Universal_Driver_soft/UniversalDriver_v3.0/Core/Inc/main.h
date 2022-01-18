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
#include "stm32f3xx_hal.h"

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
void ledBlink();
//******************
//
// DESCRIPTION:
//  Общие настройки для устройства
//
// CREATED: 13.09.2020, by Ierixon-HP
//
// FILE: main.h
//
typedef struct
{
   uint32_t BaudRate;
   uint8_t  SlaveAddress;
   uint32_t motorType;
   uint32_t Accel;
   uint32_t CurrentStop;
   uint32_t LowPWR;

}settings_t;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOC
#define CTRL_Pin GPIO_PIN_8
#define CTRL_GPIO_Port GPIOD
#define CW_CCW_Pin GPIO_PIN_11
#define CW_CCW_GPIO_Port GPIOD
#define EN_Pin GPIO_PIN_14
#define EN_GPIO_Port GPIOD
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define GD25_WP_Pin GPIO_PIN_0
#define GD25_WP_GPIO_Port GPIOD
#define GD25_HOLD_Pin GPIO_PIN_1
#define GD25_HOLD_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */


void Flash_ReadParams(settings_t *params, uint32_t source_adr);
void FLASH_WriteSettings(settings_t params, uint32_t pageAdr);

double map(double x, double in_min, double in_max, double out_min, double out_max);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
