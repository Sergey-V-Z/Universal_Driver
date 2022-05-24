/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.hpp"
#include "flash_spi.h"
#include "LED.h"

using namespace std;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern settings_t settings;
//extern TIM_HandleTypeDef htim17;
extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim8;
extern DAC_HandleTypeDef hdac;
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;

extern BLDC_motor BLDC;
extern base_motor *pMotor;
extern led LED_rs485;
extern led LED_error;
extern led LED_OSstart;

uint16_t ADC_Data[40];
//переменные для дебага
int Start = false;
bool DR = false;
uint32_t steps = 2000;
dir dir1 = dir::CW;

/* USER CODE END Variables */
osThreadId mainTaskHandle;
osThreadId Motor_poolHandle;
osThreadId NetTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// extern "C"
/* USER CODE END FunctionPrototypes */

void MainTask(void const * argument);
void motor_pool(void const * argument);
void netTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
extern "C" void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of mainTask */
  osThreadDef(mainTask, MainTask, osPriorityNormal, 0, 512);
  mainTaskHandle = osThreadCreate(osThread(mainTask), NULL);

  /* definition and creation of Motor_pool */
  osThreadDef(Motor_pool, motor_pool, osPriorityNormal, 0, 256);
  Motor_poolHandle = osThreadCreate(osThread(Motor_pool), NULL);

  /* definition and creation of NetTask */
  osThreadDef(NetTask, netTask, osPriorityNormal, 0, 512);
  NetTaskHandle = osThreadCreate(osThread(NetTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_MainTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_MainTask */
void MainTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN MainTask */
  pMotor->Init(settings);
  //pMotor->SetCurrentMax(settings.CurrentMax);
  //pMotor->SetCurrentStop(settings.CurrentStop);
  pMotor->SetPWM_Mode(settings.LowPWR);

  LED_rs485.Init(LED3_GPIO_Port, LED3_Pin);
  LED_error.Init(LED1_GPIO_Port, LED1_Pin);
  LED_OSstart.Init(LED2_GPIO_Port, LED2_Pin);
  LED_OSstart.setParameters(mode::BLINK, 2000, 100);
  LED_OSstart.LEDon();
  /* Infinite loop */
  HAL_GPIO_WritePin(GD25_HOLD_GPIO_Port, GD25_HOLD_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GD25_WP_GPIO_Port, GD25_WP_Pin, GPIO_PIN_SET);

  uint32_t tickcount = osKernelSysTick();// переменная для точной задержки

  for(;;)
  {
     LED_rs485.poll();
     LED_error.poll();
     LED_OSstart.poll();

     if(Start == 1){
        Start = 0;
        pMotor->stop();
     }
     if(Start == 2){
        Start = 0;
        pMotor->deceleration();

     }
     if(Start == 3){
        Start = 0;
        pMotor->start();

     }
     if(Start == 4){
        Start = 0;

     }

     //taskYIELD();
     osDelayUntil(&tickcount, 1); // задача будет вызываься ровро через 1 милисекунду
  }

  /* USER CODE END MainTask */
}

/* USER CODE BEGIN Header_motor_pool */
/**
* @brief Function implementing the Motor_pool thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor_pool */
void motor_pool(void const * argument)
{
  /* USER CODE BEGIN motor_pool */
	   uint32_t tickcount = osKernelSysTick();// переменная для точной задержки
	   /* Infinite loop */
	   for(;;)
	   {
	      //osDelay(1);
	      pMotor->AccelHandler();
	      osDelayUntil(&tickcount, 1); // задача будет вызываься ровро через 1 милисекунду
	   }
  /* USER CODE END motor_pool */
}

/* USER CODE BEGIN Header_netTask */
/**
* @brief Function implementing the NetTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_netTask */
void netTask(void const * argument)
{
  /* USER CODE BEGIN netTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END netTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/******************************************************************************************************
Handlers
******************************************************************************************************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == enc_Z_in_Pin){
      pMotor->SensHandler();
    }
}
/* USER CODE END Application */
