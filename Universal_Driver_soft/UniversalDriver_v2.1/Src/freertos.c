/* USER CODE BEGIN Header */
/**
******************************************************************************
* File Name          : freertos.c
* Description        : Code for freertos applications
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under Ultimate Liberty license
* SLA0044, the "License"; You may not use this file except in compliance with
* the License. You may obtain a copy of the License at:
*                             www.st.com/SLA0044
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
#include "mb.h"
#include "mbport.h"
#include "motor.hpp"
#include "flash.h"
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
extern TIM_HandleTypeDef htim17;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern DAC_HandleTypeDef hdac;
extern SPI_HandleTypeDef hspi2;

//extern step_motor stepper;
extern BLDC_motor BLDC;
extern base_motor *pMotor;
extern led LED_rs485;
extern led LED_error;
extern led LED_OSstart;

//переменные для дебага
int Start = false;
bool DR = false;
uint32_t steps = 200;
dir dir1 = dir::CW;
/* USER CODE END Variables */
osThreadId mainTaskHandle;
osThreadId modBusHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// extern "C"
/* USER CODE END FunctionPrototypes */

void MainTask(void const * argument);
void ModBus(void const * argument);

extern "C" void MX_USB_DEVICE_Init(void);
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

  /* definition and creation of modBus */
  osThreadDef(modBus, ModBus, osPriorityNormal, 0, 512);
  modBusHandle = osThreadCreate(osThread(modBus), NULL);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN MainTask */
   //    uint8_t TxBuff[10] = {0x05};
   //    uint8_t RxBuff[50] = {0};
   //    uint16_t Size = 1;
   //    HAL_StatusTypeDef StatusSPI2;
   // переподключение USB
   osDelay(20);
   HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, GPIO_PIN_SET);
   osDelay(20);
   HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, GPIO_PIN_RESET);
   
   
   pMotor->Init();
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
      pMotor->timeout++;
      if((pMotor->timeout == 30000) & (pMotor->getStatusRotation() == statusMotor::MOTION)){
         pMotor->stop();
      }
      //HAL_GPIO_TogglePin(clock_GPIO_Port, clock_Pin);
      
      // тестирование флешки
      //        HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, GPIO_PIN_RESET);
      //        StatusSPI2 = HAL_SPI_Transmit(&hspi2, TxBuff, 1, 100);
      //        HAL_Delay(100);
      //        StatusSPI2 = HAL_SPI_Receive(&hspi2, RxBuff, Size, 100);
      //        StatusSPI2 = StatusSPI2;
      //        HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, GPIO_PIN_SET);
      //        
      if(Start == 1){
         Start = 0;
         
         //         TIM8->CNT = 0;
         //         TIM2->CNT = 0;
         //         HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_4);
         
         pMotor->start();
         //         if(DR){
         //            BLDC.DirectCW(); 
         //         }else{
         //            BLDC.DirectCCW();
         //         }
         //         BLDC.start();
         
      }
      if(Start == 2){
         Start = 0;
         //         BLDC.stop();
         //HAL_TIM_OC_Stop(&htim8, TIM_CHANNEL_4);
         pMotor->deceleration();
         
      }
      if(Start == 3){
         Start = 0;
         pMotor->goTo(steps, dir1);
         
      }
      if(Start == 4){
         Start = 0;
         pMotor->goTo(steps, dir::CCW);
      }      
      //taskYIELD();
      osDelayUntil(&tickcount, 1); // задача будет вызываься ровро через 1 милисекунду
   }
  /* USER CODE END MainTask */
}

/* USER CODE BEGIN Header_ModBus */
/**
* @brief Function implementing the modBus thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ModBus */
void ModBus(void const * argument)
{
  /* USER CODE BEGIN ModBus */
   /* Infinite loop */
   eMBErrorCode eStatus = eMBInit( MB_RTU, settings.SlaveAddress, 3, settings.BaudRate, MB_PAR_NONE );
   eStatus = eMBEnable();
   //HAL_TIM_Base_Start_IT(&htim17);
   for(;;)
   {
      eMBPoll();
      taskYIELD();
   }
  /* USER CODE END ModBus */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/*description https://www.freemodbus.org/api/group__modbus__registers.html*/
//0x04
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
   eMBErrorCode    eStatus = MB_ENOERR;
   
   return eStatus;
}
//0x03 0x06 0x10
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
   //uint8_t CMD[5] = {0};
   volatile HAL_StatusTypeDef status;
   
   if(usAddress == 0 ){}
   else{usAddress--;} 
   
   eMBErrorCode    eStatus = MB_ENOERR;
   
   switch (eMode)
   {
     case MB_REG_READ:
      {	
         switch (usAddress)
         {
           case 0: //  Stop/Start
            {	
               
               break;
            }
           case 1: // Dir
            {	
               
               break;
            }
           case 2: //Status start/stop
            {	
               *pucRegBuffer = (UCHAR)BLDC.getStatusRotation();
               break;
            }
           case 3: //Status Dir
            {	
               *pucRegBuffer = (UCHAR)BLDC.getStatusDirect();
               break;
            }
           case 4: // RPM
            {	
               
               break;
            }
           case 5: // 
            {	
               break;
            }
           case 6: // 
            {	
               
               break;
            }
           case 7: // 
            {	
               break;
            }
           case 8: 
            {	
               
               break;
            }
           case 9: 
            {	
               
               break;
            }
           case 10: 
            {	
               
               break;
            }
            
           default:
            {	
               eStatus = MB_ENOREG;
               break;
            }
         }
         
         break;
      }
     case MB_REG_WRITE:
      {	
         
         switch (usAddress)
         {
           case 0: //  Stop/Start
            {	
              
               if((!*(pucRegBuffer+1)) && (pMotor->getStatusRotation() == statusMotor :: MOTION)){
                  pMotor->deceleration();
                  //osDelay(10);
                  pMotor->removeBreak(false);
               }else if(pMotor->getStatusRotation() == statusMotor :: STOPPED){
                  pMotor->removeBreak(true);
                  //osDelay(100);
                  pMotor->start();
                  
               }
               break;
            }
           case 1: // Dir
            {	
               if(!*(pucRegBuffer+1)){
                  pMotor->deceleration();
                  //osDelay(100);
                  pMotor->SetDirection(dir::CW);
                  pMotor->start();
               }else{
                  pMotor->deceleration();
                  //osDelay(100);
                  pMotor->SetDirection(dir::CCW);
                  pMotor->start();
               }
               break;
            }
           case 2: 
            {	
               
               break;
            }
           case 3: 
            {	
               
               break;
            }
           case 4: // 
            {	
               uint16_t temp = 0;
               temp = temp | (*(pucRegBuffer) << 8);
               temp = temp | *(pucRegBuffer+1);
               pMotor->setSpeed(temp);
               break;
            }
           case 5: // 
            {	
               break;
            }
           case 6: // 
            {	
               
               break;
            }
           case 7: // 
            {	
               break;
            }
           case 8: 
            {	
               FLASH_WriteSettings(settings, StartSettingsAddres);
               break;
            }
           case 9: 
            {	
               settings.BaudRate = ((*(pucRegBuffer)<<8) | (*(pucRegBuffer+1)))*10;
               break;
            }
           case 10: 
            {	
               settings.SlaveAddress = *(pucRegBuffer+1);
               break;
            }
           default:
            {	
               eStatus = MB_ENOREG;
               break;
            }
         }
         break;
      }
     default:
      {	
         eStatus = MB_EINVAL;
         break;
      }
   }
   
   return eStatus;
}

// 0x01 0x0f 0x05
eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
   return MB_ENOREG;
}
//0x02
eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
   return MB_ENOREG;
}   

/******************************************************************************************************
Handlers
******************************************************************************************************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
   //   if((GPIO_Pin == sens1_Pin)|(GPIO_Pin == sens2_Pin)){
   //      pMotor->SensHandler();
   //   }
   //   if((GPIO_Pin == zeroD_Pin) & (DR)){
   //     HAL_TIM_OC_Stop(&htim8, TIM_CHANNEL_4);
   //   }
}



/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
