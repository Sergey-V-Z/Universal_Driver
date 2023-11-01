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
void ledBlink();
//double map(double x, double in_min, double in_max, double out_min, double out_max);
typedef enum {CW, CCW, END_OF_LIST}dir;
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
	uint8_t 	ip[4];// = {192, 168, 0, 2};
	uint8_t		mask[4];//  = {255, 255, 255, 0};
	uint8_t 	gateway[4];// = {192, 168, 0, 1};
}setIP_t;

typedef struct
{
	uint8_t	MAC[6];
	setIP_t	 saveIP;
	uint8_t DHCPset;
	dir  Direct;						// направление вращения
	uint8_t  Mode_Rotation;				// режим вращения по количеству шагов или бесконечно
	uint32_t Speed;						// скорость
	uint32_t Accel;						// ускорение шагов в милисекунду
	uint32_t Slowdown;					// торможение шагов
	uint32_t SlowdownDistance;				// расстояние для торможения
	uint32_t Target;					// сколько сделать шагов до остановки
	uint32_t stepsENC;					// сколько шагов делает енкодер от одного датчика до другого
	uint32_t stepsENCtoOneStepMotor;	// сколько шагов энкодера на один шаг мотора
	uint32_t TimeOut;					// время до остановки при отстутствии движения
	uint8_t version;

}settings_t;


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MAC_IP_Pin_Pin GPIO_PIN_2
#define MAC_IP_Pin_GPIO_Port GPIOE
#define R_Pin GPIO_PIN_13
#define R_GPIO_Port GPIOC
#define G_Pin GPIO_PIN_14
#define G_GPIO_Port GPIOC
#define B_Pin GPIO_PIN_15
#define B_GPIO_Port GPIOC
#define eth_RST_Pin GPIO_PIN_0
#define eth_RST_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_8
#define EN_GPIO_Port GPIOE
#define CW_CCW_Pin GPIO_PIN_10
#define CW_CCW_GPIO_Port GPIOE
#define D0_Pin GPIO_PIN_13
#define D0_GPIO_Port GPIOD
#define D0_EXTI_IRQn EXTI15_10_IRQn
#define D1_Pin GPIO_PIN_14
#define D1_GPIO_Port GPIOD
#define D1_EXTI_IRQn EXTI15_10_IRQn
#define D2_Pin GPIO_PIN_15
#define D2_GPIO_Port GPIOD
#define D2_EXTI_IRQn EXTI15_10_IRQn
#define MAC_b7_Pin GPIO_PIN_6
#define MAC_b7_GPIO_Port GPIOC
#define MAC_b6_Pin GPIO_PIN_7
#define MAC_b6_GPIO_Port GPIOC
#define MAC_b5_Pin GPIO_PIN_8
#define MAC_b5_GPIO_Port GPIOC
#define MAC_b4_Pin GPIO_PIN_9
#define MAC_b4_GPIO_Port GPIOC
#define MAC_b3_Pin GPIO_PIN_8
#define MAC_b3_GPIO_Port GPIOA
#define MAC_b2_Pin GPIO_PIN_9
#define MAC_b2_GPIO_Port GPIOA
#define MAC_b1_Pin GPIO_PIN_10
#define MAC_b1_GPIO_Port GPIOA
#define MAC_b0_Pin GPIO_PIN_11
#define MAC_b0_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define WP_Pin GPIO_PIN_0
#define WP_GPIO_Port GPIOD
#define HOLD_Pin GPIO_PIN_1
#define HOLD_GPIO_Port GPIOD
#define enc_G_Pin GPIO_PIN_2
#define enc_G_GPIO_Port GPIOD
#define enc_notG_Pin GPIO_PIN_3
#define enc_notG_GPIO_Port GPIOD
#define enc_Z_in_Pin GPIO_PIN_3
#define enc_Z_in_GPIO_Port GPIOB
#define enc_Z_in_EXTI_IRQn EXTI3_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
