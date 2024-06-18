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
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include "api.h"
#include <iostream>
#include <vector>
#include "device_API.h"
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
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;

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

uint8_t message_rx[message_RX_LENGTH];
uint8_t UART2_rx[UART2_RX_LENGTH];
uint16_t indx_message_rx = 0;
uint16_t indx_UART2_rx = 0;
uint16_t Size_message = 0;
uint16_t Start_index = 0;

/* USER CODE END Variables */
osThreadId mainTaskHandle;
osThreadId Motor_poolHandle;
osThreadId ledTaskHandle;
osThreadId callTaskHandle;
osThreadId uart_taskHandle;
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
  osThreadDef(uart_task, uart_Task, osPriorityNormal, 0, 512);
  uart_taskHandle = osThreadCreate(osThread(uart_task), NULL);

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

	LED_IPadr.setParameters(mode::ON_OFF);
	while (gnetif.ip_addr.addr == 0) {
		osDelay(1);
	}	//ждем получение адреса
	LED_IPadr.LEDon();
	osDelay(1000);
	LED_IPadr.LEDoff();
	strIP = ip4addr_ntoa(&gnetif.ip_addr);
	printf("IP: %s\r\n", strIP.c_str());

	//структуры для netcon
	struct netconn *conn;
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
						printf("Connect open\r\n");
						while ((accept_err = netconn_recv(newconn, &netbuf))
								== ERR_OK)//работаем до тех пор пока клиент не разорвет соеденение
						{

							do {
								netbuf_data(netbuf, &in_data, &data_size);//get pointer and data size of the buffer
								in_str.assign((char*) in_data, data_size);//copy in string
								/*-----------------------------------------------------------------------------------------------------------------------------*/
								printf("Get CMD %s\r\n", in_str.c_str());

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
						printf("Connect close\r\n");
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
	pMotor->Init(&settings);
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
			pMotor->start();

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
		pMotor->Calibration_pool();
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
		osMessageGet(rxDataUART2Handle, osWaitForever);
		uint32_t message_len = strlen((char*) message_rx);
		//HAL_UART_Transmit(&huart2, message_rx, message_len, HAL_MAX_DELAY);

		// парсим  json
		cJSON *json = cJSON_Parse((char*) message_rx);
		if (json != NULL) {
			cJSON *type_data = cJSON_GetObjectItemCaseSensitive(json, "type_data");
			cJSON *save_settings = cJSON_GetObjectItemCaseSensitive(json, "save_settings");
			cJSON *obj = cJSON_GetObjectItemCaseSensitive(json, "obj");

			bool save_set = false;
			if(cJSON_IsTrue(save_settings))
			{
				save_set = true;
			}
			else
			{
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
					break;
				}
			}

			cJSON_Delete(json);
		}


		// выполняем действия

		// посылаем ответ с префиксом без префикса программа игнорирует
		osDelay(1);
	}
  /* USER CODE END uart_Task */
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
	cJSON *j_time = cJSON_GetObjectItemCaseSensitive(obj, "time");
	cJSON *j_set_time = cJSON_GetObjectItemCaseSensitive(obj, "set_time");
	cJSON *j_cw_ccw = cJSON_GetObjectItemCaseSensitive(obj, "cw_ccw");
	cJSON *j_set_cw_ccw = cJSON_GetObjectItemCaseSensitive(obj, "set_cw_ccw");
	cJSON *j_rotation = cJSON_GetObjectItemCaseSensitive(obj, "rotation");
	cJSON *j_set_rotation = cJSON_GetObjectItemCaseSensitive(obj,"set_rotation");

	if (cJSON_IsTrue(j_set_speed)) {
		if (cJSON_IsString(j_speed)) {
			pMotor->SetSpeed(std::stoi(j_speed->string));
		}

	}
	if (cJSON_IsTrue(j_set_start_speed)) {
		if (cJSON_IsString(j_start_speed)) {
			pMotor->SetStartSpeed(std::stoi(j_start_speed->string));
		}

	}
	if (cJSON_IsTrue(j_set_accel)) {
		if (cJSON_IsString(j_accel)) {
			pMotor->SetAcceleration(std::stoi(j_accel->string));
		}

	}
	if (cJSON_IsTrue(j_set_slow)) {
		if (cJSON_IsString(j_slow)) {
			pMotor->SetSlowdown(std::stoi(j_slow->string));
		}
	}
	if (cJSON_IsTrue(j_set_step_stop)) {
		if (cJSON_IsString(j_step_stop)) {
			pMotor->SetSlowdownDistance(std::stoi(j_step_stop->string));
		}
	}
	if (cJSON_IsTrue(j_set_target)) {
		if (cJSON_IsString(j_target)) {
			pMotor->SetTarget(std::stoi(j_target->string));
		}
	}
	if (cJSON_IsTrue(j_set_time)) {
		if (cJSON_IsString(j_time)) {
			pMotor->setTimeOut(std::stoi(j_time->string));
		}
	}
	if (cJSON_IsTrue(j_set_cw_ccw)) {
		if (cJSON_IsFalse(j_cw_ccw)) {
			pMotor->SetDirection(dir::CW);
		}
		else
		{
			pMotor->SetDirection(dir::CCW);
		}
	}
	if (cJSON_IsTrue(j_set_rotation)) {
		if (cJSON_IsString(j_rotation)) {

			if(strcasecmp(j_rotation->string, "Infiniti"))
			{
				pMotor->SetMode(mode_rotation_t::infinity);

			}
			else if(strcasecmp(j_rotation->string, "Continue ENC"))
			{
				pMotor->SetMode(mode_rotation_t::by_meter_enc);

			}
			else if(strcasecmp(j_rotation->string, "Continue Counter"))
			{
				pMotor->SetMode(mode_rotation_t::by_meter_timer);

			}
			else if(strcasecmp(j_rotation->string, "Infiniti ENC"))
			{
				pMotor->SetMode(mode_rotation_t::infinity_enc);
			}
			else
			{

			}
		}
	}

	// сохранение
	if (save) {
		mem_spi.W25qxx_EraseSector(0);
		osDelay(5);
		mem_spi.Write(settings);
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

	if (cJSON_IsTrue(j_setIP)) {
		char sep = '.';
		std::string s = j_IP->string;
		if (!s.empty()) {
			std::string sepIP[4];
			int i = 0;

			for (size_t p = 0, q = 0; (p != s.npos) || (i < 4); p = q, i++)
				sepIP[i] = s.substr(p + (p != 0),
						(q = s.find(sep, p + 1)) - p - (p != 0));

			settings.saveIP.ip[0] = std::stoi(sepIP[0].c_str());
			settings.saveIP.ip[1] = std::stoi(sepIP[1].c_str());
			settings.saveIP.ip[2] = std::stoi(sepIP[2].c_str());
			settings.saveIP.ip[3] = std::stoi(sepIP[3].c_str());
		}

		//settings.saveIP.ip[0] = std::stoi();
	}

	if (cJSON_IsTrue(j_setMAC)) {
		char sep = ':';
		std::string s = j_MAC->string;
		if (!s.empty()) {
			std::string sepMAC[6];
			int i = 0;

			for (size_t p = 0, q = 0; (p != s.npos) || (i < 6); p = q, i++)
				sepMAC[i] = s.substr(p + (p != 0),
						(q = s.find(sep, p + 1)) - p - (p != 0));

			settings.MAC[0] = std::stoi(sepMAC[0].c_str());
			settings.MAC[1] = std::stoi(sepMAC[1].c_str());
			settings.MAC[2] = std::stoi(sepMAC[2].c_str());
			settings.MAC[3] = std::stoi(sepMAC[3].c_str());
			settings.MAC[4] = std::stoi(sepMAC[4].c_str());
			settings.MAC[5] = std::stoi(sepMAC[5].c_str());
		}
	}

	if (cJSON_IsTrue(j_setGATEWAY)) {
		char sep = '.';
		std::string s = j_GATEWAY->string;
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

	if (cJSON_IsTrue(j_setMASK)) {
		char sep = '.';
		std::string s = j_MASK->string;
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

	if (cJSON_IsTrue(j_setDNS)) {
		char sep = '.';
		std::string s = j_DNS->string;
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

	if (cJSON_IsTrue(j_setDHCP)) {
		if (cJSON_IsTrue(j_DHCP)) {
			settings.DHCPset = 1;
		} else {
			settings.DHCPset = 0;
		}
	}

	// сохранение
	if (save) {
		mem_spi.W25qxx_EraseSector(0);
		osDelay(5);
		mem_spi.Write(settings);
	}

	// отправить ответ на хост
}

void actoin_resp_all_set() {

	cJSON *j_all_settings_obj = cJSON_CreateObject();
	cJSON *j_to_host = cJSON_CreateObject();

	cJSON_AddNumberToObject(j_to_host, "type_data", 3);

	// настройки в строку
	string srtIP_to_host = std::to_string(settings.saveIP.ip[0])+"."+
							std::to_string(settings.saveIP.ip[1])+"."+
							std::to_string(settings.saveIP.ip[2])+ "."+
							std::to_string(settings.saveIP.ip[3]);
	cJSON_AddStringToObject(j_all_settings_obj, "IP", srtIP_to_host.c_str());

	string srtMAC_to_host = std::to_string(settings.MAC[0])+":"+
							std::to_string(settings.MAC[1])+":"+
							std::to_string(settings.MAC[2])+":"+
							std::to_string(settings.MAC[3])+":"+
							std::to_string(settings.MAC[4])+":"+
							std::to_string(settings.MAC[5])+":";
	cJSON_AddStringToObject(j_all_settings_obj, "MAC", srtMAC_to_host.c_str());

	string srtGATEWAY_to_host = std::to_string(settings.saveIP.gateway[0])+"."+
							std::to_string(settings.saveIP.gateway[1])+"."+
							std::to_string(settings.saveIP.gateway[2])+"."+
							std::to_string(settings.saveIP.gateway[3])+".";
	cJSON_AddStringToObject(j_all_settings_obj, "GATEWAY", srtGATEWAY_to_host.c_str());

	string srtMASK_to_host = std::to_string(settings.saveIP.mask[0])+"."+
							std::to_string(settings.saveIP.mask[1])+"."+
							std::to_string(settings.saveIP.mask[2])+"."+
							std::to_string(settings.saveIP.mask[3])+".";
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
	string srt_speed_to_host = std::to_string(pMotor->getSpeed());
	cJSON_AddStringToObject(j_all_settings_obj, "speed", srt_speed_to_host.c_str());

	string srt_accel_to_host = std::to_string(pMotor->getAcceleration());
	cJSON_AddStringToObject(j_all_settings_obj, "accel", srt_accel_to_host.c_str());

	string srt_slow_to_host = std::to_string(pMotor->getSlowdown());
	cJSON_AddStringToObject(j_all_settings_obj, "slow", srt_slow_to_host.c_str());

	string srt_step_stop_to_host = std::to_string(pMotor->getSlowdownDistance());
	cJSON_AddStringToObject(j_all_settings_obj, "step_stop", srt_step_stop_to_host.c_str());

	string srt_target_to_host = std::to_string(pMotor->getTarget());
	cJSON_AddStringToObject(j_all_settings_obj, "target", srt_target_to_host.c_str());

	string srt_time_to_host = std::to_string(pMotor->getTimeOut());
	cJSON_AddStringToObject(j_all_settings_obj, "time", srt_time_to_host.c_str());

	if(pMotor->getStatusDirect() == dir::CW)
	{
		cJSON_AddTrueToObject(j_all_settings_obj, "cw_ccw");
	}
	else
	{
		cJSON_AddFalseToObject(j_all_settings_obj, "cw_ccw");
	}

	switch (pMotor->getMode()) {
		case mode_rotation_t::infinity:
			cJSON_AddStringToObject(j_all_settings_obj, "rotation", "Infiniti");
			break;
		case mode_rotation_t::infinity_enc:
			cJSON_AddStringToObject(j_all_settings_obj, "rotation", "Infiniti ENC");
			break;
		case mode_rotation_t::by_meter_timer:
			cJSON_AddStringToObject(j_all_settings_obj, "rotation", "Continue Counter");
			break;
		case mode_rotation_t::by_meter_enc:
			cJSON_AddStringToObject(j_all_settings_obj, "rotation", "Continue ENC");
			break;
		default:
			cJSON_AddStringToObject(j_all_settings_obj, "rotation", "");
			break;
	}

	string srt_version_to_host = std::to_string(settings.version);
	cJSON_AddStringToObject(j_all_settings_obj, "version", srt_version_to_host.c_str());


	cJSON_AddItemToObject(j_to_host, "obj", j_all_settings_obj);

	char *str_to_host = cJSON_Print(j_to_host);
	string out = str_to_host;
	out += '\r';

	HAL_UART_Transmit(&huart2, (uint8_t*)out.c_str(), out.size(), HAL_MAX_DELAY);

	cJSON_free(str_to_host);
	cJSON_Delete(j_to_host);
}

void actoin_resp_status() {

	cJSON *j_all_settings_obj = cJSON_CreateObject();
	cJSON *j_to_host = cJSON_CreateObject();

	cJSON_AddNumberToObject(j_to_host, "type_data", 4);

	string srt_l_dist = std::to_string(pMotor->getLastDistance());
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
	string out = str_to_host;
	out += '\r';

	HAL_UART_Transmit(&huart2, (uint8_t*)out.c_str(), out.size(), HAL_MAX_DELAY);

	cJSON_free(str_to_host);
	cJSON_Delete(j_to_host);
}

/******************************************************************************************************
 Handlers
 ******************************************************************************************************/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART2) {

		while ( __HAL_UART_GET_FLAG(huart, UART_FLAG_TC) != SET) {
		};

		uint16_t Size_Data = Size - Start_index;

		HAL_UART_RxEventTypeTypeDef rxEventType;
		rxEventType = HAL_UARTEx_GetRxEventType(huart);
		switch (rxEventType) {
		case HAL_UART_RXEVENT_IDLE:
			//printf( "IDLE. Size:%d sd:%d sti:%d\n\r ", Size, Size_Data, Start_index);
			// копировать с индекса сообщения
			memcpy(&message_rx[indx_message_rx], &UART2_rx[Start_index],
					Size_Data);

			//|| (message_rx[indx_message_rx + Size_Data - 1] == '\n')
			if ((message_rx[indx_message_rx + Size_Data - 1] == '\r')
					|| (message_rx[indx_message_rx + Size_Data - 1] == 0)) {
				message_rx[indx_message_rx + Size_Data] = 0;
				// выдать сигнал
				osMessagePut(rxDataUART2Handle, (uint32_t) indx_message_rx, 0);
				Size_message = 0;
				// обнулить индекс сообщения
				indx_message_rx = 0;
			} else {
				indx_message_rx += Size_Data;
			}

			Start_index = Size;

			//printf( "\n\r" );
			break;

		case HAL_UART_RXEVENT_HT:
			//printf( "HT Size:%d sd:%d sti:%d\n\r", Size, Size_Data, Start_index);
			break;

		case HAL_UART_RXEVENT_TC:
			//printf( "TC Size:%d sd:%d sti:%d\n\r", Size, Size_Data, Start_index);
			// скопировать в начало буфера
			memcpy(&message_rx[indx_message_rx], &UART2_rx[Start_index],
					Size_Data);
			// сохронить индекс сообщения
			indx_message_rx += Size_Data;
			Start_index = 0;
			break;

		default:
			printf("???\n\r");
			break;
		}

		HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_rx, UART2_RX_LENGTH);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
		//usart_rx_check(Size);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if ((GPIO_Pin == D0_Pin) || (GPIO_Pin == D1_Pin)) {
		pMotor->SensHandler(GPIO_Pin);
	}

	if (GPIO_Pin == enc_Z_in_Pin) {
		//pMotor->SensHandler();
	}
}
/* USER CODE END Application */
