/**
  ******************************************************************************
  * @file    comp.c
  * @brief   This file provides code for the configuration
  *          of the COMP instances.
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

/* Includes ------------------------------------------------------------------*/
#include "comp.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

COMP_HandleTypeDef hcomp3;
COMP_HandleTypeDef hcomp6;
COMP_HandleTypeDef hcomp7;

/* COMP3 init function */
void MX_COMP3_Init(void)
{

  /* USER CODE BEGIN COMP3_Init 0 */

  /* USER CODE END COMP3_Init 0 */

  /* USER CODE BEGIN COMP3_Init 1 */

  /* USER CODE END COMP3_Init 1 */
  hcomp3.Instance = COMP3;
  hcomp3.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH2;
  hcomp3.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO2;
  hcomp3.Init.Output = COMP_OUTPUT_NONE;
  hcomp3.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp3.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp3.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
  hcomp3.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp3.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp3.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP3_Init 2 */

  /* USER CODE END COMP3_Init 2 */

}
/* COMP6 init function */
void MX_COMP6_Init(void)
{

  /* USER CODE BEGIN COMP6_Init 0 */

  /* USER CODE END COMP6_Init 0 */

  /* USER CODE BEGIN COMP6_Init 1 */

  /* USER CODE END COMP6_Init 1 */
  hcomp6.Instance = COMP6;
  hcomp6.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH2;
  hcomp6.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp6.Init.Output = COMP_OUTPUT_NONE;
  hcomp6.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp6.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp6.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
  hcomp6.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp6.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp6.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP6_Init 2 */

  /* USER CODE END COMP6_Init 2 */

}
/* COMP7 init function */
void MX_COMP7_Init(void)
{

  /* USER CODE BEGIN COMP7_Init 0 */

  /* USER CODE END COMP7_Init 0 */

  /* USER CODE BEGIN COMP7_Init 1 */

  /* USER CODE END COMP7_Init 1 */
  hcomp7.Instance = COMP7;
  hcomp7.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH2;
  hcomp7.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp7.Init.Output = COMP_OUTPUT_NONE;
  hcomp7.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp7.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp7.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
  hcomp7.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp7.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp7.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP7_Init 2 */

  /* USER CODE END COMP7_Init 2 */

}

void HAL_COMP_MspInit(COMP_HandleTypeDef* compHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(compHandle->Instance==COMP3)
  {
  /* USER CODE BEGIN COMP3_MspInit 0 */

  /* USER CODE END COMP3_MspInit 0 */

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**COMP3 GPIO Configuration
    PD14     ------> COMP3_INP
    PA8     ------> COMP3_OUT
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_COMP3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN COMP3_MspInit 1 */

  /* USER CODE END COMP3_MspInit 1 */
  }
  else if(compHandle->Instance==COMP6)
  {
  /* USER CODE BEGIN COMP6_MspInit 0 */

  /* USER CODE END COMP6_MspInit 0 */

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**COMP6 GPIO Configuration
    PD11     ------> COMP6_INP
    PA10     ------> COMP6_OUT
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_COMP6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN COMP6_MspInit 1 */

  /* USER CODE END COMP6_MspInit 1 */
  }
  else if(compHandle->Instance==COMP7)
  {
  /* USER CODE BEGIN COMP7_MspInit 0 */

  /* USER CODE END COMP7_MspInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**COMP7 GPIO Configuration
    PA0     ------> COMP7_INP
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN COMP7_MspInit 1 */

  /* USER CODE END COMP7_MspInit 1 */
  }
}

void HAL_COMP_MspDeInit(COMP_HandleTypeDef* compHandle)
{

  if(compHandle->Instance==COMP3)
  {
  /* USER CODE BEGIN COMP3_MspDeInit 0 */

  /* USER CODE END COMP3_MspDeInit 0 */

    /**COMP3 GPIO Configuration
    PD14     ------> COMP3_INP
    PA8     ------> COMP3_OUT
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_14);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

  /* USER CODE BEGIN COMP3_MspDeInit 1 */

  /* USER CODE END COMP3_MspDeInit 1 */
  }
  else if(compHandle->Instance==COMP6)
  {
  /* USER CODE BEGIN COMP6_MspDeInit 0 */

  /* USER CODE END COMP6_MspDeInit 0 */

    /**COMP6 GPIO Configuration
    PD11     ------> COMP6_INP
    PA10     ------> COMP6_OUT
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_11);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_10);

  /* USER CODE BEGIN COMP6_MspDeInit 1 */

  /* USER CODE END COMP6_MspDeInit 1 */
  }
  else if(compHandle->Instance==COMP7)
  {
  /* USER CODE BEGIN COMP7_MspDeInit 0 */

  /* USER CODE END COMP7_MspDeInit 0 */

    /**COMP7 GPIO Configuration
    PA0     ------> COMP7_INP
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

  /* USER CODE BEGIN COMP7_MspDeInit 1 */

  /* USER CODE END COMP7_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
