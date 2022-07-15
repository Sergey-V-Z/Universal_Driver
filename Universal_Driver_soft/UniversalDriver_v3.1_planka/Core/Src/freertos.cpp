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
#include "lwip.h"
using namespace std;
#include <string>
#include "api.h"
#include <iostream>
#include <vector>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct mesage_t{
	uint32_t cmd;
	uint32_t addres_var;
	uint32_t data_in;
	bool need_resp;
	bool data_in_is;
	uint32_t data_out;
	string err; // сообщение клиенту об ошибке в сообщении
	bool f_bool; // наличие ошибки в сообшении
};

enum class pos_t{position1, position1_2, position2};
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
extern TIM_HandleTypeDef htim1;
extern DAC_HandleTypeDef hdac;
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
//extern SPI_HandleTypeDef hspi3;

//extern step_motor stepper;
extern base_motor *pMotor;
extern led LED_IPadr;
extern led LED_error;
extern led LED_OSstart;

uint16_t ADC_Data[40];
//переменные для дебага
int Start = false;
bool DR = false;
uint32_t steps = 2000;
dir dir1 = dir::CW;

settings_t settings = {115200, 0x0D, 0};
bool resetSettings = false;

// for SPI Flash
extern SPI_HandleTypeDef hspi3;
pins_spi_t ChipSelect = {SPI3_CS_GPIO_Port, SPI3_CS_Pin};
pins_spi_t WriteProtect = {WP_GPIO_Port, WP_Pin};
pins_spi_t Hold = {HOLD_GPIO_Port, HOLD_Pin};

flash mem_spi;

//структуры для netcon
extern struct netif gnetif;

//TCP_IP
string strIP;
string in_str;

//переменные для обшей работы
uint32_t var_sys[100];
pos_t position = pos_t::position1_2; // содержит текущее положение планки
uint32_t watchdog = 3000; // максимальное время выполнения операции а милисек
bool needCall = true; // необходимость калибровки
bool permission_calibrate = false; // разрешение на калибровку

/* USER CODE END Variables */
osThreadId mainTaskHandle;
osThreadId Motor_poolHandle;
osThreadId ledTaskHandle;
osThreadId MotorActionHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// extern "C"
/* USER CODE END FunctionPrototypes */

void MainTask(void const * argument);
void motor_pool(void const * argument);
void LedTask(void const * argument);
void motor_action(void const * argument);

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

  /* definition and creation of ledTask */
  osThreadDef(ledTask, LedTask, osPriorityNormal, 0, 128);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* definition and creation of MotorAction */
  osThreadDef(MotorAction, motor_action, osPriorityNormal, 0, 256);
  MotorActionHandle = osThreadCreate(osThread(MotorAction), NULL);

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

	mem_spi.Init(&hspi3, 0, ChipSelect, WriteProtect, Hold);

	mem_spi.Read(&settings);
	if((settings.BaudRate == 0) | (settings.BaudRate == 0xFFFFFFFF) | resetSettings)
	{
		resetSettings = false;
		settings.BaudRate = 115200;
		settings.SlaveAddress = 0x02;
		settings.motorType = 0;
		settings.Accel = 100;
		settings.Deaccel = 100;
		settings.CurrentStop = 200;
		settings.LowPWR = 1;
		mem_spi.Write(settings);
	}

	LED_IPadr.setParameters(mode::ON_OFF);
	while(gnetif.ip_addr.addr == 0){osDelay(1);}	//ждем получение адреса
	LED_IPadr.LEDon();
	osDelay(1000);
	LED_IPadr.LEDoff();
	strIP = ip4addr_ntoa(&gnetif.ip_addr);

	//структуры для netcon
	struct netconn *conn;
	struct netconn *newconn;
	struct netbuf *netbuf;
	volatile err_t err, accept_err;
	//ip_addr_t local_ip;
	//ip_addr_t remote_ip;
	void 		*in_data = NULL;
	uint16_t 		data_size = 0;

	//Флаги для разбора сообщения
	string f_cmd("C");
	string f_addr("A");
	string f_datd("D");
	string delim("x");

	/* Infinite loop */
	for(;;)
	{

		conn = netconn_new(NETCONN_TCP);
		if (conn!=NULL)
		{
			err = netconn_bind(conn,NULL,81);//assign port number to connection
			if (err==ERR_OK)
			{
				netconn_listen(conn);//set port to listening mode
				while(1)
				{
					accept_err=netconn_accept(conn,&newconn);//suspend until new connection
					if (accept_err==ERR_OK)
					{
						LED_IPadr.LEDon();
						while ((accept_err=netconn_recv(newconn,&netbuf))==ERR_OK)//работаем до тех пор пока клиент не разорвет соеденение
						{

							do
							{
								netbuf_data(netbuf,&in_data,&data_size);//get pointer and data size of the buffer
								in_str.assign((char*)in_data, data_size);//copy in string
/*-----------------------------------------------------------------------------------------------------------------------------*/
								// Парсинг
								vector<string> arr_msg;
								vector<mesage_t> arr_cmd;
								size_t prev = 0;
								size_t next;
								size_t delta = delim.length();

								//разбить на сообщения
								while( ( next = in_str.find( delim, prev ) ) != string::npos ){
									arr_msg.push_back( in_str.substr( prev, (next +1)-prev ) );
									prev = next + delta;
								}
								//arr_msg.push_back( in_str.substr( prev ) );

								//занести сообщения в структуру
								int count_msg = arr_msg.size();
								for (int i = 0; i < count_msg; ++i) {
									prev = 0;
									next = 0;
									size_t posC = 0;
									//size_t posA = 0;
									size_t posD = 0;
									size_t posx = 0;
									mesage_t temp_msg;

									// выделение комманды
									delta = f_cmd.length();
									next = arr_msg[i].find(f_cmd);
									posC = next;
									if(next == string::npos){
										//Ошибка
										temp_msg.err = "wrong format in C flag";
										temp_msg.f_bool = true;
										arr_cmd.push_back(temp_msg);
										continue;

									}
									prev = next + delta;
/*
									// выделение адреса
									delta = f_addr.length();
									next = arr_msg[i].find(f_addr, prev);
									posA = next;
									if(next == string::npos){
										//Ошибка
										temp_msg.err = "wrong format in A flag";
										temp_msg.f_bool = true;
										arr_cmd.push_back(temp_msg);
										continue;
									}
									prev = next + delta;
*/
									// выделение данных
									delta = f_datd.length();
									next = arr_msg[i].find(f_datd, prev);
									posD = next;
									if(next == string::npos){
										//Ошибка
										temp_msg.err = "wrong format in D flag";
										temp_msg.f_bool = true;
										arr_cmd.push_back(temp_msg);
										continue;
									}
									prev = next + delta;

									// выделение данных
									delta = delim.length();
									next = arr_msg[i].find(delim, prev);
									posx = next;
									if(next == string::npos){
										//Ошибка
										temp_msg.err = "wrong format in x flag";
										temp_msg.f_bool = true;
										arr_cmd.push_back(temp_msg);
										continue;
									}

									temp_msg.cmd = (uint32_t)stoi(arr_msg[i].substr(posC +1, (posD -1) - posC));
									//temp_msg.addres_var = (uint32_t)stoi(arr_msg[i].substr(posA +1, (posD -1) - posA));
									temp_msg.data_in = (uint32_t)stoi(arr_msg[i].substr(posD +1, (posx -1) - posD));
									arr_cmd.push_back(temp_msg);
								}
								// Закончили парсинг
/*-----------------------------------------------------------------------------------------------------------------------------*/
								//Выполнение комманд
								int count_cmd = arr_cmd.size();
								for (int i = 0; i < count_cmd; ++i) {
									uint32_t temp;
									switch (arr_cmd[i].cmd) {
										case 1:
											arr_cmd[i].data_out = (uint32_t)pMotor->getStatusDirect();
											arr_cmd[i].need_resp = true;
											break;
										case 2:

											temp = arr_cmd[i].data_in;
											arr_cmd[i].err = "OK";
											break;
										case 3:
											if((!arr_cmd[i].data_in) && pMotor->getStatusRotation() == statusMotor :: STOPPED){
												pMotor->SetDirection(dir::CW);
											}else if(pMotor->getStatusRotation() == statusMotor :: STOPPED){
												pMotor->SetDirection(dir::CCW);
											}
											arr_cmd[i].err = "OK";
											break;
										case 4:
											//pMotor->SetFeedbackTarget(arr_cmd[i].data_in);
											arr_cmd[i].err = "OK";
											break;
										case 5:
											//pMotor->SetZeroPoint();
											arr_cmd[i].err = "OK";
											break;
										case 6:
											//pMotor->SetSpeed(arr_cmd[i].data_in);
											arr_cmd[i].err = "OK";
											break;
										case 7:

											pMotor->SetAcceleration(arr_cmd[i].data_in);
											settings.Accel = arr_cmd[i].data_in;
											mem_spi.Write(settings);
											arr_cmd[i].err = "OK";
											break;
										case 8:
											//pMotor->SetDeacceleration(arr_cmd[i].data_in);
											settings.Deaccel = arr_cmd[i].data_in;
											mem_spi.Write(settings);
											arr_cmd[i].err = "OK";
											break;
										case 9:
											//pMotor->SetPWRstatus((bool)arr_cmd[i].data_in);
											settings.LowPWR = arr_cmd[i].data_in;
											mem_spi.Write(settings);
											arr_cmd[i].err = "OK";
											break;
										case 10:
											mem_spi.Write(settings);
											arr_cmd[i].err = "OK";
											break;
										case 11:
											arr_cmd[i].err = "no_CMD";
											break;
										case 12:
											arr_cmd[i].err = "no_CMD";
											break;
										case 13:
											arr_cmd[i].err = "no_CMD";
											break;
										case 14:
											arr_cmd[i].err = "no_CMD";
											break;

										default:
											arr_cmd[i].err = "err_CMD";
											break;
									}
								}
/*-----------------------------------------------------------------------------------------------------------------------------*/
								//Формируем ответ
								string resp;
								for (int i = 0; i < count_cmd; ++i) {
									resp.append("C" + to_string(arr_cmd[i].cmd));
									if(arr_cmd[i].need_resp){
										resp.append("D" + to_string(arr_cmd[i].cmd));
									}else{
										resp.append("D" + arr_cmd[i].err);
									}
									resp.append("x");
								}
								netconn_write(newconn, resp.c_str(), resp.size(), NETCONN_COPY);

							} while (netbuf_next(netbuf) >= 0);
							netbuf_delete(netbuf);

						}
						netconn_close(newconn);
						netconn_delete(newconn);
						LED_IPadr.LEDoff();
					} else netconn_delete(newconn);
					osDelay(20);
				}
			}
		}
		osDelay(1);
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
	pMotor->Init(settings);
	//pMotor->SetCurrentMax(settings.CurrentMax);
	//pMotor->SetCurrentStop(settings.CurrentStop);
	//pMotor->SetPWM_Mode(settings.LowPWR);
	//uint32_t tickcount = osKernelSysTick();// переменная для точной задержки

	/* Infinite loop */
	for(;;)
	{

		//osDelay(1);
		//pMotor->AccelHandler();
		//osDelayUntil(&tickcount, 1); // задача будет вызываься ровро через 1 милисекунду
		osDelay(1);
	}
  /* USER CODE END motor_pool */
}

/* USER CODE BEGIN Header_LedTask */
/**
 * @brief Function implementing the ledTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LedTask */
void LedTask(void const * argument)
{
  /* USER CODE BEGIN LedTask */

	LED_IPadr.Init(G_GPIO_Port, G_Pin);
	LED_error.Init(R_GPIO_Port, R_Pin);
	LED_OSstart.Init(B_GPIO_Port, B_Pin);
	LED_OSstart.setParameters(mode::BLINK, 2000, 100);
	LED_OSstart.LEDon();

	HAL_GPIO_WritePin(HOLD_GPIO_Port, HOLD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(WP_GPIO_Port, WP_Pin, GPIO_PIN_SET);

	//uint32_t tickcount = osKernelSysTick();// переменная для точной задержки
	/* Infinite loop */
	for(;;)
	{
		LED_IPadr.poll();
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
			pMotor->SetDirection(dir::CW);
		}
		if(Start == 5){
			Start = 0;
			pMotor->SetDirection(dir::CCW);
		}

		osDelay(1);
		//taskYIELD();
		//osDelayUntil(&tickcount, 1); // задача будет вызываься ровро через 1 милисекунду
	}
  /* USER CODE END LedTask */
}

/* USER CODE BEGIN Header_motor_action */
/**
* @brief Function implementing the MotorAction thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor_action */
void motor_action(void const * argument)
{
  /* USER CODE BEGIN motor_action */

	uint32_t stepsAll = 0;
	uint32_t time;
  /* Infinite loop */
  for(;;)
  {
	  if(needCall && permission_calibrate){

		  uint8_t bos_bit = 0;
		  bos_bit |= ((HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin)) << 0) | ((HAL_GPIO_ReadPin(D2_GPIO_Port, D2_Pin)) << 1);

		  switch (bos_bit) {
			case 0:
				position = pos_t::position1_2;
				break;
			case 1:
				position = pos_t::position1;
				break;
			case 2:
				position = pos_t::position2;
				break;
			default:
				//err
				break;
		}
/*			// Определить текущее положение
			if((HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin)) || !(HAL_GPIO_ReadPin(D2_GPIO_Port, D2_Pin))){
				position = pos_t::position1;
			}else if((HAL_GPIO_ReadPin(D2_GPIO_Port, D2_Pin)) || !(HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin))){
				position = pos_t::position2;
			}else{
				position = pos_t::position1_2;
			}*/

		  switch (position) {
		  // начать движение в точку 2 и считать количество сделанных шагов
		  case (pos_t::position1):{
			  pMotor->SetDirection(dir::CCW);
			  // сбросить счетчик CNT ARR в максимум что бы не было прерывания
			  // запустить watchdog операции
			  // запустить движение и ожидать в цикле одно из двух событий(срабатывание датчика или watchdog)
			  // остановка происходит по прерыванию от датчика или watchdog

			  pMotor->start();
			  for  (time = 0; ((time <= watchdog) && (position != pos_t::position2)); ++time) {// ожидаем прихода в точку 2 или watchdog
				  osDelay(1);
			  }
			  if(time >= watchdog){
				  pMotor->stop();
				  permission_calibrate  = false;
				  // ошибка выполнения операции
			  }else if (position == pos_t::position2){
				  // прибыли на место закончили калибровку
				  needCall = false;
				  permission_calibrate  = false;
				  // сохранить измеренное количество шагов
				  // расчитать места для торможения ускорения
			  }
			  break;
		  }

		  // начать движение в точку 1
		  case (pos_t::position1_2):{
			  pMotor->SetDirection(dir::CW);

			  pMotor->start();
			  for  (time = 0; ((time <= watchdog) && (position != pos_t::position1)); ++time) {// ожидаем прихода в точку 2 или watchdog
				  osDelay(1);
			  }
			  if(time >= watchdog){
				  pMotor->stop();
				  permission_calibrate  = false;
				  // ошибка выполнения операции
			  }else if (position == pos_t::position1){

			  }
			  break;
		  }

		  // начать движение в точку 1
		  case (pos_t::position2):{
			  pMotor->SetDirection(dir::CW);

			  pMotor->start();
			  for  (time = 0; ((time <= watchdog) && (position != pos_t::position1)); ++time) {// ожидаем прихода в точку 2 или watchdog
				  osDelay(1);
			  }
			  if(time >= watchdog){
				  pMotor->stop();
				  permission_calibrate  = false;
				  // ошибка выполнения операции
			  }else if (position == pos_t::position1){

			  }
			  break;
		  }

		  default:
			  break;
		  }
	  }
	  osDelay(1);
  }
  /* USER CODE END motor_action */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/******************************************************************************************************
Handlers
 ******************************************************************************************************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	   if((GPIO_Pin == D1_Pin)){
	      pMotor->stop();
	      position = pos_t::position1;
	   }
	   if((GPIO_Pin == D2_Pin)){
	      pMotor->stop();
	      position = pos_t::position2;
	   }
	   pMotor->SensHandler(GPIO_Pin);

}
/* USER CODE END Application */
