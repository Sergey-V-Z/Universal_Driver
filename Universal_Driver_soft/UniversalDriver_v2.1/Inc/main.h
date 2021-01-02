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
   
}settings_t;


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN_SW1_Pin GPIO_PIN_4
#define IN_SW1_GPIO_Port GPIOE
#define IN_SW2_Pin GPIO_PIN_5
#define IN_SW2_GPIO_Port GPIOE
#define IN_SW3_Pin GPIO_PIN_6
#define IN_SW3_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOC
#define EN_hard_Pin GPIO_PIN_4
#define EN_hard_GPIO_Port GPIOC
#define Home_Pin GPIO_PIN_5
#define Home_GPIO_Port GPIOC
#define CTRL_Pin GPIO_PIN_0
#define CTRL_GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_1
#define EN_GPIO_Port GPIOB
#define RESET_Pin GPIO_PIN_2
#define RESET_GPIO_Port GPIOB
#define H_F_Pin GPIO_PIN_7
#define H_F_GPIO_Port GPIOE
#define CW_CCW_Pin GPIO_PIN_8
#define CW_CCW_GPIO_Port GPIOE
#define SW_s_h_Pin GPIO_PIN_10
#define SW_s_h_GPIO_Port GPIOE
#define EN_soft_Pin GPIO_PIN_15
#define EN_soft_GPIO_Port GPIOE
#define GD25_HOLD_Pin GPIO_PIN_10
#define GD25_HOLD_GPIO_Port GPIOB
#define GD25_WP_Pin GPIO_PIN_11
#define GD25_WP_GPIO_Port GPIOB
#define SPI2_CS1_Pin GPIO_PIN_12
#define SPI2_CS1_GPIO_Port GPIOB
#define SW_encDiff_Pin GPIO_PIN_10
#define SW_encDiff_GPIO_Port GPIOD
#define USB_EN_Pin GPIO_PIN_9
#define USB_EN_GPIO_Port GPIOA
#define RELE1_Pin GPIO_PIN_0
#define RELE1_GPIO_Port GPIOD
#define RELE2_Pin GPIO_PIN_1
#define RELE2_GPIO_Port GPIOD
#define RELE3_Pin GPIO_PIN_2
#define RELE3_GPIO_Port GPIOD
#define hal3_Pin GPIO_PIN_3
#define hal3_GPIO_Port GPIOD
#define hal2_Pin GPIO_PIN_4
#define hal2_GPIO_Port GPIOD
#define G__Pin GPIO_PIN_5
#define G__GPIO_Port GPIOD
#define G_Pin GPIO_PIN_6
#define G_GPIO_Port GPIOD
#define hal1_Pin GPIO_PIN_7
#define hal1_GPIO_Port GPIOD
#define enc_Z_in_Pin GPIO_PIN_3
#define enc_Z_in_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */



void Flash_ReadParams(settings_t *params, uint32_t source_adr);
void FLASH_WriteSettings(settings_t params, uint32_t pageAdr);

double map(double x, double in_min, double in_max, double out_min, double out_max);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
