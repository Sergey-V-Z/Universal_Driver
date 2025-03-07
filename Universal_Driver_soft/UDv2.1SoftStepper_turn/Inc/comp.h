/**
  ******************************************************************************
  * File Name          : COMP.h
  * Description        : This file provides code for the configuration
  *                      of the COMP instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __comp_H
#define __comp_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern COMP_HandleTypeDef hcomp3;
extern COMP_HandleTypeDef hcomp6;
extern COMP_HandleTypeDef hcomp7;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_COMP3_Init(void);
void MX_COMP6_Init(void);
void MX_COMP7_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ comp_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
