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
extern TIM_HandleTypeDef htim8;
extern DAC_HandleTypeDef hdac;

extern step_motor stepper;
//extern BLDC_motor BLDC;

extern base_motor *pMotor;

int Start = false;
bool DR = false;
int i = 0;
/* USER CODE END Variables */
osThreadId mainTaskHandle;
osThreadId modBusHandle;
osSemaphoreId myBinarySem01Handle;
osSemaphoreId myBinarySem02Handle;
osSemaphoreId myBinarySem03Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// extern "C"
/* USER CODE END FunctionPrototypes */

void MainTask(void const * argument);
void ModBus(void const * argument);

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

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

  /* definition and creation of myBinarySem02 */
  osSemaphoreDef(myBinarySem02);
  myBinarySem02Handle = osSemaphoreCreate(osSemaphore(myBinarySem02), 1);

  /* definition and creation of myBinarySem03 */
  osSemaphoreDef(myBinarySem03);
  myBinarySem03Handle = osSemaphoreCreate(osSemaphore(myBinarySem03), 1);

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
  /* USER CODE BEGIN MainTask */
   
   // switch control to BLDC 
   HAL_GPIO_WritePin(select_GPIO_Port, select_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(OE_hand_GPIO_Port, OE_hand_Pin, GPIO_PIN_SET);

   // switch control to STEP 
//   HAL_GPIO_WritePin(select_GPIO_Port, select_Pin, GPIO_PIN_RESET);
//   HAL_GPIO_WritePin(OE_step_drv_GPIO_Port, OE_step_drv_Pin, GPIO_PIN_SET); 
//   HAL_GPIO_WritePin(OE_hand_GPIO_Port, OE_hand_Pin, GPIO_PIN_SET);
   //stepper.Init();
   //pMotor->goTo(10, dir::CW);
   //pMotor->endMotionHandler();
   /* Infinite loop */
   for(;;)
   {
     //HAL_GPIO_TogglePin(clock_GPIO_Port, clock_Pin);
      
      HAL_GPIO_WritePin(EN_A_B_GPIO_Port, EN_A_B_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(EN_C_D_GPIO_Port, EN_C_D_Pin, GPIO_PIN_SET);
     
      for(i = 0; i < 100; ++i)
      {
         HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_SET);
         osDelay(3);
         HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
         osDelay(3);
         HAL_GPIO_WritePin(IN2_B_GPIO_Port, IN2_B_Pin, GPIO_PIN_SET);
         osDelay(3);
         HAL_GPIO_WritePin(IN2_B_GPIO_Port, IN2_B_Pin, GPIO_PIN_RESET);
         osDelay(3);  
         HAL_GPIO_WritePin(IN3_C_GPIO_Port, IN3_C_Pin, GPIO_PIN_SET);
         osDelay(3);
         HAL_GPIO_WritePin(IN3_C_GPIO_Port, IN3_C_Pin, GPIO_PIN_RESET);
         osDelay(3);
         
      }

      if(Start == 1){
         Start = 0;
         
         TIM8->CNT = 0;
         TIM2->CNT = 0;
         HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_4);
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
         
      }
      
      taskYIELD();
     //osDelay(1);
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
   HAL_TIM_Base_Start_IT(&htim17);
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
               //*pucRegBuffer = (UCHAR)BLDC.getStatusRotation();
               break;
            }
           case 3: //Status Dir
            {	
               //*pucRegBuffer = (UCHAR)BLDC.getStatusDirect();
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
               if(!*(pucRegBuffer+1)){
                  pMotor->stop();
                  //osDelay(10);
                  pMotor->removeBreak(false);
               }else{
                  pMotor->removeBreak(true);
                  //osDelay(100);
                  pMotor->start();
                  
               }
               break;
            }
           case 1: // Dir
            {	
               if(!*(pucRegBuffer+1)){
                  pMotor->stop();
                  //osDelay(100);
                  pMotor->SetDirection(dir::CW);
                  pMotor->start();
               }else{
                  pMotor->stop();
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
               pMotor->setSpeed(*(pucRegBuffer+1));
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
   if((GPIO_Pin == sens1_Pin)|(GPIO_Pin == sens2_Pin)|(GPIO_Pin == sens3_Pin) ){
      //BLDC.SensHandler();
   }
}



/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
