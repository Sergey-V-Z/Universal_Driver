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
#include "flash_spi.h"
#include "LED.h"
#include "lwip.h"
using namespace std;
#include <string>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include "api.h"
#include <iostream>
#include <vector>
#include "device_API.h"
#include <iostream>
#include <iomanip>
#include "externDriver.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//for ethernetif.c
//extern settings_t settings;
//MACAddr[5] = settings.MAC_end;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern settings_t settings;
//extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim3;
//extern DAC_HandleTypeDef hdac;
//extern DMA_HandleTypeDef hdma_adc1;
//extern ADC_HandleTypeDef hadc1;

extern extern_driver *pMotor;
extern led LED_IPadr;
extern led LED_error;
extern led LED_OSstart;

void actoin_motor_set(cJSON *obj, bool save);
void actoin_ip(cJSON *obj, bool save);
void actoin_resp_all_set(void);
void actoin_resp_status(void);

uint16_t ADC_Data[40];
//переменные для дебага
int Start = false;
bool DR = false;
uint32_t steps = 2000;
//dir dir1 = dir::CW;

extern settings_t settings;

// for SPI Flash
extern SPI_HandleTypeDef hspi3;

extern flash mem_spi;

//структуры для netcon
extern struct netif gnetif;

//TCP_IP
string strIP;
string in_str;

//переменные для обшей работы
uint32_t var_sys[100];

extern uint8_t message_rx[message_RX_LENGTH];
extern uint8_t UART2_rx[UART2_RX_LENGTH];
extern uint16_t indx_message_rx;
extern uint16_t indx_UART2_rx;
extern uint16_t Size_message;
extern uint16_t Start_index;
/* USER CODE END Variables */
osThreadId mainTaskHandle;
osThreadId Motor_poolHandle;
osThreadId ledTaskHandle;
osThreadId callTaskHandle;
osThreadId uart_taskHandle;
osThreadId loggerTaskHandle;
osMessageQId rxDataUART2Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// extern "C"
/* USER CODE END FunctionPrototypes */

void MainTask(void const * argument);
void motor_pool(void const * argument);
void LedTask(void const * argument);
void CallTask(void const * argument);
void uart_Task(void const * argument);
void LoggerTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
extern "C" void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
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

  /* Create the queue(s) */
  /* definition and creation of rxDataUART2 */
  osMessageQDef(rxDataUART2, 16, uint8_t);
  rxDataUART2Handle = osMessageCreate(osMessageQ(rxDataUART2), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of mainTask */
  osThreadDef(mainTask, MainTask, osPriorityNormal, 0, 512);
  mainTaskHandle = osThreadCreate(osThread(mainTask), NULL);

  /* definition and creation of Motor_pool */
  osThreadDef(Motor_pool, motor_pool, osPriorityNormal, 0, 512);
  Motor_poolHandle = osThreadCreate(osThread(Motor_pool), NULL);

  /* definition and creation of ledTask */
  osThreadDef(ledTask, LedTask, osPriorityNormal, 0, 512);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* definition and creation of callTask */
  osThreadDef(callTask, CallTask, osPriorityNormal, 0, 128);
  callTaskHandle = osThreadCreate(osThread(callTask), NULL);

  /* definition and creation of uart_task */
  osThreadDef(uart_task, uart_Task, osPriorityNormal, 0, 1024);
  uart_taskHandle = osThreadCreate(osThread(uart_task), NULL);

  /* definition and creation of loggerTask */
  osThreadDef(loggerTask, LoggerTask, osPriorityNormal, 0, 128);
  loggerTaskHandle = osThreadCreate(osThread(loggerTask), NULL);

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
  //нициализируем логгер
  Logger_Init(&huart2);

  STM_LOG("Start step enc. %f", 0.01);

	LED_IPadr.setParameters(mode::ON_OFF);
	while (gnetif.ip_addr.addr == 0) {
		osDelay(1);
	}	//ждем получение адреса
	LED_IPadr.LEDon();
	osDelay(1000);
	LED_IPadr.LEDoff();
	strIP = ip4addr_ntoa(&gnetif.ip_addr);
	STM_LOG("IP: %s", strIP.c_str());

	//структуры для netcon
	struct netconn *conn = NULL;
	struct netconn *newconn;
	struct netbuf *netbuf;
	volatile err_t err, accept_err;
	//ip_addr_t local_ip;
	//ip_addr_t remote_ip;
	void *in_data = NULL;
	uint16_t data_size = 0;

	//Флаги для разбора сообщения
	string f_cmd("C");
	string f_addr("A");
	string f_datd("D");
	string delim("x");

	/* Infinite loop */
	for (;;) {

		conn = netconn_new(NETCONN_TCP);
		if (conn != NULL) {
			err = netconn_bind(conn, NULL, 81);	//assign port number to connection
			if (err == ERR_OK) {
				netconn_listen(conn);	//set port to listening mode
				while (1) {
					accept_err = netconn_accept(conn, &newconn);//suspend until new connection
					if (accept_err == ERR_OK) {
						LED_IPadr.LEDon();
						STM_LOG("Connect open");
						while ((accept_err = netconn_recv(newconn, &netbuf))
								== ERR_OK)//работаем до тех пор пока клиент не разорвет соеденение
						{

							do {
								netbuf_data(netbuf, &in_data, &data_size);//get pointer and data size of the buffer
								in_str.assign((char*) in_data, data_size);//copy in string
								/*-----------------------------------------------------------------------------------------------------------------------------*/
								STM_LOG("Get CMD %s", in_str.c_str());

								if(!in_str.empty())
								{
									string resp = Command_execution(in_str);
									netconn_write(newconn, resp.c_str(), resp.size(), NETCONN_COPY);
								}

							} while (netbuf_next(netbuf) >= 0);
							netbuf_delete(netbuf);

						}
						netconn_close(newconn);
						netconn_delete(newconn);
						STM_LOG("Connect close");
						LED_IPadr.LEDoff();
					} else
						netconn_delete(newconn);
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
	pMotor->Init();
	//uint32_t tickcount = osKernelSysTick();// переменная для точной задержки
	/* Infinite loop */
	for (;;) {
		//osDelay(1);
		pMotor->AccelHandler();

		//osDelayUntil(&tickcount, 1); // задача будет вызываься ровно через 1 милисекунду
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
	LED_OSstart.setParameters(mode::BLINK, 1000, 100);
	LED_OSstart.LEDon();

	//uint32_t tickcount = osKernelSysTick();// переменная для точной задержки
	/* Infinite loop */
	for (;;) {
		LED_IPadr.poll();
		LED_error.poll();
		LED_OSstart.poll();

		if (Start == 1) {
			Start = 0;
			pMotor->stop(statusTarget_t::finished);
		}
		if (Start == 2) {
			Start = 0;
			pMotor->slowdown();

		}
		if (Start == 3) {
			Start = 0;
			pMotor->start(pMotor->getTarget());

		}
		if (Start == 4) {
			Start = 0;

		}

		osDelay(1);
		//taskYIELD();
		//osDelayUntil(&tickcount, 1); // задача будет вызываься ровро через 1 милисекунду
	}
  /* USER CODE END LedTask */
}

/* USER CODE BEGIN Header_CallTask */
/**
 * @brief Function implementing the callTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CallTask */
void CallTask(void const * argument)
{
  /* USER CODE BEGIN CallTask */

	/* Infinite loop */
	for (;;) {
		if(pMotor->Calibration_pool())
		{

            // Сохраняем настройки в память
            //mem_spi.W25qxx_EraseSector(0);
            //osDelay(5);
            //mem_spi.Write(settings);

            STM_LOG("Calibration completed successfully");
		}
		//pMotor->findHome();
		osDelay(1);
	}
  /* USER CODE END CallTask */
}

/* USER CODE BEGIN Header_uart_Task */
/**
 * @brief Function implementing the uart_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_uart_Task */
void uart_Task(void const * argument)
{
  /* USER CODE BEGIN uart_Task */
	//HAL_UART_Receive_DMA(&huart2, UART2_rx, UART2_RX_LENGTH);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_rx, UART2_RX_LENGTH);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	//__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_TC);
	/* Infinite loop */
	for (;;) {
		// ожидать собщение
		osEvent evt = osMessageGet(rxDataUART2Handle, 60000);

		if (evt.status == osEventMessage) {
		    // Сообщение успешно получено
		    //uint32_t received = evt.value.v;
		} else {
		    // Обработка ошибки
			STM_LOG("wait message timeout");
			continue;
		}
		//uint32_t message_len = strlen((char*) message_rx);
		//HAL_UART_Transmit(&huart2, message_rx, message_len, HAL_MAX_DELAY);

		// парсим  json
		cJSON *json = cJSON_Parse((char*) message_rx);
		if (json != NULL) {

			cJSON *id = cJSON_GetObjectItemCaseSensitive(json, "id");
			cJSON *name_device = cJSON_GetObjectItemCaseSensitive(json, "name_device");
			cJSON *type_data = cJSON_GetObjectItemCaseSensitive(json, "type_data");
			cJSON *save_settings = cJSON_GetObjectItemCaseSensitive(json, "save_settings");
			cJSON *obj = cJSON_GetObjectItemCaseSensitive(json, "obj");

			if (cJSON_IsNumber(id) && cJSON_GetNumberValue(id) == ID_CTRL) {
				bool save_set = false;
				if (cJSON_IsTrue(save_settings)) {
					save_set = true;
				} else {
					save_set = false;
				}

				if (cJSON_IsNumber(type_data)) {
					switch (type_data->valueint) {
					case 1: // ip settings
						actoin_ip(obj, save_set);
						break;
					case 2: // motor settings
						actoin_motor_set(obj, save_set);
						break;
					case 3:
						actoin_resp_all_set();
						break;
					case 4:
						actoin_resp_status();
						break;
					default:
						STM_LOG("data type not registered");
						break;
					}
				}
			} else {
				STM_LOG("id not valid");
			}

			cJSON_Delete(json);
		} else {
			STM_LOG("Invalid JSON");
		}

		osDelay(1);
	}
  /* USER CODE END uart_Task */
}

/* USER CODE BEGIN Header_LoggerTask */
/**
* @brief Function implementing the loggerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LoggerTask */
void LoggerTask(void const * argument)
{
  /* USER CODE BEGIN LoggerTask */
  /* Infinite loop */
  for(;;)
  {
	Logger_Process();
    osDelay(1);
  }
  /* USER CODE END LoggerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void actoin_motor_set(cJSON *obj, bool save) {
	cJSON *j_speed = cJSON_GetObjectItemCaseSensitive(obj, "speed");
	cJSON *j_set_speed = cJSON_GetObjectItemCaseSensitive(obj, "set_speed");
	cJSON *j_start_speed = cJSON_GetObjectItemCaseSensitive(obj, "start_speed");
	cJSON *j_set_start_speed = cJSON_GetObjectItemCaseSensitive(obj, "set_start_speed");
	cJSON *j_accel = cJSON_GetObjectItemCaseSensitive(obj, "accel");
	cJSON *j_set_accel = cJSON_GetObjectItemCaseSensitive(obj, "set_accel");
	cJSON *j_slow = cJSON_GetObjectItemCaseSensitive(obj, "slow");
	cJSON *j_set_slow = cJSON_GetObjectItemCaseSensitive(obj, "set_slow");
	cJSON *j_step_stop = cJSON_GetObjectItemCaseSensitive(obj, "step_stop");
	cJSON *j_set_step_stop = cJSON_GetObjectItemCaseSensitive(obj,"set_step_stop");
	cJSON *j_target = cJSON_GetObjectItemCaseSensitive(obj, "target");
	cJSON *j_set_target = cJSON_GetObjectItemCaseSensitive(obj, "set_target");
	cJSON *j_time_out = cJSON_GetObjectItemCaseSensitive(obj, "time_out");
	cJSON *j_set_time_out = cJSON_GetObjectItemCaseSensitive(obj, "set_time_out");
	cJSON *j_motor = cJSON_GetObjectItemCaseSensitive(obj, "motor");
	cJSON *j_set_motor = cJSON_GetObjectItemCaseSensitive(obj, "set_motor");
	cJSON *j_cw_ccw = cJSON_GetObjectItemCaseSensitive(obj, "cw_ccw");
	cJSON *j_set_cw_ccw = cJSON_GetObjectItemCaseSensitive(obj, "set_cw_ccw");
	cJSON *j_rotation = cJSON_GetObjectItemCaseSensitive(obj, "rotation");
	cJSON *j_set_rotation = cJSON_GetObjectItemCaseSensitive(obj,"set_rotation");

	// ловим исключения
	// записать новые данные в во временную переменную и после проверки на исключение выставить в настройки

	int Speed = 0;
	int StartSpeed = 0;
	int Acceleration = 0;
	int Slowdown = 0;
	int SlowdownDistance = 0;
	int Target = 0;
	int TimeOut = 0;

	try {

		if ((j_set_speed != NULL) && (j_speed != NULL) && cJSON_IsString(j_speed))
			if (cJSON_IsTrue(j_set_speed))
				Speed = std::stoi(j_speed->valuestring);
			else {
			}
		else
			throw(0);

		if ((j_set_start_speed != NULL) && (j_start_speed != NULL) && cJSON_IsString(j_start_speed))
			if (cJSON_IsTrue(j_set_start_speed))
				StartSpeed = std::stoi(j_start_speed->valuestring);
			else {
			}
		else
			throw(0);

		if ((j_set_accel != NULL) && (j_accel != NULL) && cJSON_IsString(j_accel))
			if (cJSON_IsTrue(j_set_accel))
				Acceleration = std::stoi(j_accel->valuestring);
			else {
			}
		else
			throw(0);

		if ((j_set_slow != NULL) && (j_slow != NULL) && cJSON_IsString(j_slow))
			if (cJSON_IsTrue(j_set_slow))
				Slowdown = std::stoi(j_slow->valuestring);
			else {
			}
		else
			throw(0);

		if ((j_set_step_stop != NULL) && (j_step_stop != NULL) && cJSON_IsString(j_step_stop))
			if (cJSON_IsTrue(j_set_step_stop))
				SlowdownDistance = std::stoi(j_step_stop->valuestring);
			else {
			}
		else
			throw(0);

		if ((j_set_target != NULL) && (j_target != NULL) && cJSON_IsString(j_target))
			if (cJSON_IsTrue(j_set_target))
				Target = std::stoi(j_target->valuestring);
			else {
			}
		else
			throw(0);

		if ((j_set_time_out != NULL) && (j_time_out != NULL) && cJSON_IsString(j_time_out))
			if (cJSON_IsTrue(j_set_time_out))
				TimeOut = std::stoi(j_time_out->valuestring);
			else {
			}
		else
			throw(0);

		// запись в настройки
		if (cJSON_IsTrue(j_set_speed))
			pMotor->SetSpeed(Speed);

		if (cJSON_IsTrue(j_set_start_speed))
			pMotor->SetStartSpeed(StartSpeed);

		if (cJSON_IsTrue(j_set_accel))
			pMotor->SetAcceleration(Acceleration);

		if (cJSON_IsTrue(j_set_slow))
			pMotor->SetSlowdown(Slowdown);

		if (cJSON_IsTrue(j_set_step_stop))
			pMotor->SetSlowdownDistance(SlowdownDistance);

		if (cJSON_IsTrue(j_set_target))
			pMotor->SetTarget(Target);

		if (cJSON_IsTrue(j_set_time_out))
			pMotor->setTimeOut(TimeOut);

		if ((j_set_motor != NULL) && (j_motor != NULL) && cJSON_IsNumber(j_motor)) {
			if (cJSON_IsTrue(j_set_motor)) {
				// проверить на диапазон
				if (j_motor->valuedouble >= motor_t::stepper_motor && j_motor->valuedouble <= motor_t::bldc)
					pMotor->SetMotor((motor_t)j_motor->valuedouble);
				else
					pMotor->SetMotor(motor_t::stepper_motor);
			}

		}

		if ((j_set_cw_ccw != NULL) && (j_cw_ccw != NULL) && cJSON_IsNumber(j_cw_ccw)) {
			if (cJSON_IsTrue(j_set_cw_ccw)) {
				// проверить на диапазон
				if (j_cw_ccw->valuedouble >= dir::CW && j_cw_ccw->valuedouble <= dir::CCW)
					pMotor->SetDirection((dir)j_cw_ccw->valuedouble);
				else
					pMotor->SetDirection(dir::CW);
			}

		}

		if ((j_set_rotation != NULL) && (j_rotation != NULL) && cJSON_IsNumber(j_rotation)) {
			if (cJSON_IsTrue(j_set_rotation)) {
				pMotor->SetMode((mode_rotation_t)j_rotation->valuedouble);
			}
		}

		STM_LOG("Settings motor set successful");
		// сохранение
		if (save) {
			STM_LOG("Save motor settings");
			mem_spi.W25qxx_EraseSector(0);
			osDelay(5);
			mem_spi.Write(settings);
		}
	} catch (...) {
		//ex.what()
		STM_LOG("err argument in motor parametrs");
		//return;
	}

}

void actoin_ip(cJSON *obj, bool save) {
	cJSON *j_IP = cJSON_GetObjectItemCaseSensitive(obj, "IP");
	cJSON *j_setIP = cJSON_GetObjectItemCaseSensitive(obj, "setIP");
	cJSON *j_MAC = cJSON_GetObjectItemCaseSensitive(obj, "MAC");
	cJSON *j_setMAC = cJSON_GetObjectItemCaseSensitive(obj, "setMAC");
	cJSON *j_GATEWAY = cJSON_GetObjectItemCaseSensitive(obj, "GATEWAY");
	cJSON *j_setGATEWAY = cJSON_GetObjectItemCaseSensitive(obj, "setGATEWAY");
	cJSON *j_MASK = cJSON_GetObjectItemCaseSensitive(obj, "MASK");
	cJSON *j_setMASK = cJSON_GetObjectItemCaseSensitive(obj, "setMASK");
	cJSON *j_DNS = cJSON_GetObjectItemCaseSensitive(obj, "DNS");
	cJSON *j_setDNS = cJSON_GetObjectItemCaseSensitive(obj, "setDNS");
	cJSON *j_DHCP = cJSON_GetObjectItemCaseSensitive(obj, "DHCP");
	cJSON *j_setDHCP = cJSON_GetObjectItemCaseSensitive(obj, "setDHCP");

	try {
		if ((j_setIP != NULL) && cJSON_IsTrue(j_setIP)) {
			char sep = '.';
			std::string s = j_IP->valuestring;
			if (!s.empty()) {
				std::string sepIP[4];
				int i = 0;

				for (size_t p = 0, q = 0; (p != s.npos) || (i < 4); p = q, i++)
					sepIP[i] = s.substr(p + (p != 0),
							(q = s.find(sep, p + 1)) - p - (p != 0));

				// записать новые данные в во временную переменную и после проверки на исключение выставить в настройки
				settings.saveIP.ip[0] = std::stoi(sepIP[0].c_str());
				settings.saveIP.ip[1] = std::stoi(sepIP[1].c_str());
				settings.saveIP.ip[2] = std::stoi(sepIP[2].c_str());
				settings.saveIP.ip[3] = std::stoi(sepIP[3].c_str());
			}

			//settings.saveIP.ip[0] = std::stoi();
		}

		if ((j_setMAC != NULL) && cJSON_IsTrue(j_setMAC)) {
			char sep = ':';
			std::string s = j_MAC->valuestring;
			if (!s.empty()) {
				std::string sepMAC[6];
				int i = 0;

				for (size_t p = 0, q = 0; (p != s.npos) || (i < 6); p = q, i++)
					sepMAC[i] = s.substr(p + (p != 0),
							(q = s.find(sep, p + 1)) - p - (p != 0));

				size_t pos = 0;
				settings.MAC[0] = std::stoi(sepMAC[0].c_str(), &pos, 16);
				settings.MAC[1] = std::stoi(sepMAC[1].c_str(), &pos, 16);
				settings.MAC[2] = std::stoi(sepMAC[2].c_str(), &pos, 16);
				settings.MAC[3] = std::stoi(sepMAC[3].c_str(), &pos, 16);
				settings.MAC[4] = std::stoi(sepMAC[4].c_str(), &pos, 16);
				settings.MAC[5] = std::stoi(sepMAC[5].c_str(), &pos, 16);
			}
		}

		if ((j_setGATEWAY != NULL) && cJSON_IsTrue(j_setGATEWAY)) {
			char sep = '.';
			std::string s = j_GATEWAY->valuestring;
			if (!s.empty()) {
				std::string sepGATEWAY[4];
				int i = 0;

				for (size_t p = 0, q = 0; (p != s.npos) || (i < 4); p = q, i++)
					sepGATEWAY[i] = s.substr(p + (p != 0),
							(q = s.find(sep, p + 1)) - p - (p != 0));

				settings.saveIP.gateway[0] = std::stoi(sepGATEWAY[0].c_str());
				settings.saveIP.gateway[1] = std::stoi(sepGATEWAY[1].c_str());
				settings.saveIP.gateway[2] = std::stoi(sepGATEWAY[2].c_str());
				settings.saveIP.gateway[3] = std::stoi(sepGATEWAY[3].c_str());
			}
		}

		if ((j_setMASK != NULL) && cJSON_IsTrue(j_setMASK)) {
			char sep = '.';
			std::string s = j_MASK->valuestring;
			if (!s.empty()) {
				std::string sepMASK[4];
				int i = 0;

				for (size_t p = 0, q = 0; (p != s.npos) || (i < 4); p = q, i++)
					sepMASK[i] = s.substr(p + (p != 0),
							(q = s.find(sep, p + 1)) - p - (p != 0));

				settings.saveIP.mask[0] = std::stoi(sepMASK[0].c_str());
				settings.saveIP.mask[1] = std::stoi(sepMASK[1].c_str());
				settings.saveIP.mask[2] = std::stoi(sepMASK[2].c_str());
				settings.saveIP.mask[3] = std::stoi(sepMASK[3].c_str());
			}
		}

		if ((j_setDNS != NULL) && cJSON_IsTrue(j_setDNS)) {
			char sep = '.';
			std::string s = j_DNS->valuestring;
			if (!s.empty()) {
				std::string sepDNS[4];
				int i = 0;

				for (size_t p = 0, q = 0; (p != s.npos) || (i < 4); p = q, i++)
					sepDNS[i] = s.substr(p + (p != 0),
							(q = s.find(sep, p + 1)) - p - (p != 0));

				//settings.[0] = std::stoi(sepDNS[0].c_str());
				//settings.[1] = std::stoi(sepDNS[1].c_str());
				//settings.[2] = std::stoi(sepDNS[2].c_str());
				//settings.[3] = std::stoi(sepDNS[3].c_str());

			}

		}

		if ((j_setDHCP != NULL) && cJSON_IsTrue(j_setDHCP)) {
			if (cJSON_IsTrue(j_DHCP)) {
				settings.DHCPset = 1;
			} else {
				settings.DHCPset = 0;
			}
		}

		STM_LOG("Settings set successful");
		// сохранение
		if (save) {
			STM_LOG("Save settings");
			mem_spi.W25qxx_EraseSector(0);
			osDelay(5);
			mem_spi.Write(settings);
		}

		// отправить ответ на хост
	} catch (...) {
		//ex.what()
		STM_LOG("err argument in motor parametrs");
		//return;
	}
}

void actoin_resp_all_set() {

	cJSON *j_all_settings_obj = cJSON_CreateObject();
	if (!j_all_settings_obj) return;

	cJSON *j_to_host = cJSON_CreateObject();
	if (!j_to_host) return;

	cJSON_AddNumberToObject(j_to_host, "id", ID_CTRL);
	cJSON_AddStringToObject(j_to_host, "name_device", NAME);
	cJSON_AddNumberToObject(j_to_host, "type_data", 3);

	// настройки в строку
	string srtIP_to_host = std::to_string(settings.saveIP.ip[0])+"."+
							std::to_string(settings.saveIP.ip[1])+"."+
							std::to_string(settings.saveIP.ip[2])+ "."+
							std::to_string(settings.saveIP.ip[3]);
	cJSON_AddStringToObject(j_all_settings_obj, "IP", srtIP_to_host.c_str());

	char srtMAC_to_host[100];
	snprintf(srtMAC_to_host, sizeof(srtMAC_to_host),
            "%02x:%02x:%02x:%02x:%02x:%02x",
			settings.MAC[0],settings.MAC[1],settings.MAC[2],
			settings.MAC[3],settings.MAC[4],settings.MAC[5]);

	cJSON_AddStringToObject(j_all_settings_obj, "MAC", srtMAC_to_host);

	string srtGATEWAY_to_host = std::to_string(settings.saveIP.gateway[0])+"."+
							std::to_string(settings.saveIP.gateway[1])+"."+
							std::to_string(settings.saveIP.gateway[2])+"."+
							std::to_string(settings.saveIP.gateway[3]);
	cJSON_AddStringToObject(j_all_settings_obj, "GATEWAY", srtGATEWAY_to_host.c_str());

	string srtMASK_to_host = std::to_string(settings.saveIP.mask[0])+"."+
							std::to_string(settings.saveIP.mask[1])+"."+
							std::to_string(settings.saveIP.mask[2])+"."+
							std::to_string(settings.saveIP.mask[3]);
	cJSON_AddStringToObject(j_all_settings_obj, "MASK", srtMASK_to_host.c_str());

	cJSON_AddStringToObject(j_all_settings_obj, "DNS", "0.0.0.0");

	if(settings.DHCPset)
	{
		cJSON_AddTrueToObject(j_all_settings_obj, "DHCP");
	}
	else
	{
		cJSON_AddFalseToObject(j_all_settings_obj, "DHCP");
	}

	// настройки мотора
	cJSON_AddStringToObject(j_all_settings_obj, "speed", std::to_string(pMotor->getSpeed()).c_str());

	cJSON_AddStringToObject(j_all_settings_obj, "start_speed", std::to_string(pMotor->getStartSpeed()).c_str());

	cJSON_AddStringToObject(j_all_settings_obj, "accel", std::to_string(pMotor->getAcceleration()).c_str());

	cJSON_AddStringToObject(j_all_settings_obj, "slow", std::to_string(pMotor->getSlowdown()).c_str());

	cJSON_AddStringToObject(j_all_settings_obj, "step_stop", std::to_string(pMotor->getSlowdownDistance()).c_str());

	cJSON_AddStringToObject(j_all_settings_obj, "target", std::to_string(pMotor->getTarget()).c_str());

	cJSON_AddStringToObject(j_all_settings_obj, "time_out", std::to_string(pMotor->getTimeOut()).c_str());

	cJSON_AddNumberToObject(j_all_settings_obj, "motor", pMotor->getMotor());

	cJSON_AddNumberToObject(j_all_settings_obj, "cw_ccw", pMotor->getStatusDirect());

	cJSON_AddNumberToObject(j_all_settings_obj, "rotation", pMotor->getMode());

	cJSON_AddStringToObject(j_all_settings_obj, "version", std::to_string(settings.version).c_str());


	cJSON_AddItemToObject(j_to_host, "obj", j_all_settings_obj);

	char *str_to_host = cJSON_Print(j_to_host);

	STM_LOG("%s", str_to_host);

	cJSON_free(str_to_host);
	cJSON_Delete(j_to_host);
	//cJSON_Delete(j_all_settings_obj);
}

void actoin_resp_status() {

	cJSON *j_all_settings_obj = cJSON_CreateObject();
	if (!j_all_settings_obj) return;

	cJSON *j_to_host = cJSON_CreateObject();
	if (!j_to_host) return;

	cJSON_AddNumberToObject(j_to_host, "id", ID_CTRL);
	cJSON_AddStringToObject(j_to_host, "name_device", NAME);
	cJSON_AddNumberToObject(j_to_host, "type_data", 4);

	string srt_l_dist = std::to_string(pMotor->getCurrentSteps());
	cJSON_AddStringToObject(j_all_settings_obj, "l_dist", srt_l_dist.c_str());

	switch (pMotor->getStatusRotation()) {
		case statusMotor::ACCEL:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_rotation", "ACCEL");
			break;
		case statusMotor::BRAKING:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_rotation", "BRAKING");
			break;
		case statusMotor::MOTION:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_rotation", "MOTION");
			break;
		case statusMotor::STOPPED:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_rotation", "STOPPED");
			break;
		default:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_rotation", "");
			break;
	}

	switch (pMotor->getStatusTarget()) {
		case statusTarget_t::inProgress:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_target", "inProgress");
			break;
		case statusTarget_t::errMotion:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_target", "errMotion");
			break;
		case statusTarget_t::errDirection:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_target", "errDirection");
			break;
		case statusTarget_t::finished:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_target", "finished");
			break;
		default:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_target", "");
			break;
	}

	switch (pMotor->get_pos()) {
		case pos_t::D0:
			cJSON_AddStringToObject(j_all_settings_obj, "sensor_pos", "D0");
			break;
		case pos_t::D1:
			cJSON_AddStringToObject(j_all_settings_obj, "sensor_pos", "D1");
			break;
		case pos_t::D_0_1:
			cJSON_AddStringToObject(j_all_settings_obj, "sensor_pos", "D_0_1");
			break;
		default:
			cJSON_AddStringToObject(j_all_settings_obj, "sensor_pos", "");
			break;
	}


	cJSON_AddItemToObject(j_to_host, "obj", j_all_settings_obj);

	char *str_to_host = cJSON_Print(j_to_host);
	//string out = str_to_host;
	//out += '\r';

	//HAL_UART_Transmit(&huart2, (uint8_t*)out.c_str(), out.size(), HAL_MAX_DELAY);
	STM_LOG("%s", str_to_host);

	cJSON_free(str_to_host);
	cJSON_Delete(j_to_host);
}

/******************************************************************************************************
 Handlers
 ******************************************************************************************************/

/* USER CODE END Application */
