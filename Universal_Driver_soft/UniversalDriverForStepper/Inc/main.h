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
   uint32_t pointCW;
   uint32_t pointCCW;
   uint32_t maxSpeedDiv;
   uint32_t positionX;
   uint32_t speed;
   uint32_t accel;
}settings_t;


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Brake_Pin GPIO_PIN_14
#define Brake_GPIO_Port GPIOC
#define RED_Pin GPIO_PIN_1
#define RED_GPIO_Port GPIOC
#define GREEN_Pin GPIO_PIN_2
#define GREEN_GPIO_Port GPIOC
#define BLUE_Pin GPIO_PIN_3
#define BLUE_GPIO_Port GPIOC
#define ENC_Pin GPIO_PIN_6
#define ENC_GPIO_Port GPIOA
#define ENA_Pin GPIO_PIN_7
#define ENA_GPIO_Port GPIOA
#define IN3_C_Pin GPIO_PIN_4
#define IN3_C_GPIO_Port GPIOC
#define IN2_B_Pin GPIO_PIN_5
#define IN2_B_GPIO_Port GPIOC
#define ENB_Pin GPIO_PIN_0
#define ENB_GPIO_Port GPIOB
#define IN1_A_Pin GPIO_PIN_1
#define IN1_A_GPIO_Port GPIOB
#define select_Pin GPIO_PIN_2
#define select_GPIO_Port GPIOB
#define OE_hand_Pin GPIO_PIN_10
#define OE_hand_GPIO_Port GPIOB
#define OE_step_drv_Pin GPIO_PIN_12
#define OE_step_drv_GPIO_Port GPIOB
#define HOME_Pin GPIO_PIN_13
#define HOME_GPIO_Port GPIOB
#define CTRL_Pin GPIO_PIN_14
#define CTRL_GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_15
#define EN_GPIO_Port GPIOB
#define RESET_Pin GPIO_PIN_6
#define RESET_GPIO_Port GPIOC
#define H_F_Pin GPIO_PIN_7
#define H_F_GPIO_Port GPIOC
#define CW_CCW_Pin GPIO_PIN_8
#define CW_CCW_GPIO_Port GPIOC
#define D1_Pin GPIO_PIN_8
#define D1_GPIO_Port GPIOA
#define D1_EXTI_IRQn EXTI9_5_IRQn
#define D2_Pin GPIO_PIN_9
#define D2_GPIO_Port GPIOA
#define D2_EXTI_IRQn EXTI9_5_IRQn
#define sens1_Pin GPIO_PIN_15
#define sens1_GPIO_Port GPIOA
#define sens2_Pin GPIO_PIN_10
#define sens2_GPIO_Port GPIOC
#define sens3_Pin GPIO_PIN_11
#define sens3_GPIO_Port GPIOC
#define CS_Pin GPIO_PIN_6
#define CS_GPIO_Port GPIOB
#define WP_Pin GPIO_PIN_7
#define WP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define ENA_PWM TIM3->CCR2
#define ENB_PWM TIM3->CCR3
#define ENC_PWM TIM3->CCR1

void Flash_ReadParams(settings_t *params, uint32_t source_adr);
void FLASH_WriteSettings(settings_t params, uint32_t pageAdr);

double map(double x, double in_min, double in_max, double out_min, double out_max);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
