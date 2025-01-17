#include "main.h"
#include "cmsis_os.h"
#include "Delay_us_DWT.h"
#include "stdlib.h"

#define DIRECT_CCW HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
#define DIRECT_CW HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);

//enum class dir{CW, CCW, END_OF_LIST};
typedef enum {
	HALF, FULL
} step;
typedef enum {
	inProgress = 0, finished, errMotion, errDirection
} statusTarget_t;
typedef enum {
	MOTION, STOPPED, ACCEL, BRAKING
} statusMotor;
typedef enum {
	ENCODER, HALLSENSOR, NON
} fb;
typedef enum {
	OK, No_Connect, No_Signal
} sensorsERROR;
enum class pos_t {
	D0, D_0_1, D1
};


//******************
// CLASS: stp_motor
//
// DESCRIPTION:
//  step motor driver
//
// CREATED: 20.09.2020, by Ierixon-HP
//
// FILE: extern_driver.h
//
class extern_driver {
public:
	extern_driver(settings_t *set, TIM_HandleTypeDef *timCount,TIM_HandleTypeDef *timFreq, uint32_t channelFreq, TIM_HandleTypeDef *timDebounce, TIM_HandleTypeDef *timENC);
	~extern_driver();

	//methods for set
	void SetDirection(dir direction);
	void SetSpeed(uint32_t percent);
	void SetStartSpeed(uint32_t percent);
	void SetAcceleration(uint32_t percent);
	void SetSlowdown(uint32_t accel);
	void SetSlowdownDistance(uint32_t steps);
	uint32_t SetTarget (uint32_t Target);
	void setTimeOut(uint32_t time);
	void SetZeroPoint (void);
	void SetMode(mode_rotation_t mod);
	void SetMotor(motor_t m);
	void Parameter_update(void);

	//methods for get
	pos_t get_pos();
	uint32_t getAcceleration();
	uint32_t getSlowdown();
	uint32_t getSlowdownDistance();
	uint32_t getSpeed();
	uint32_t getStartSpeed();
	uint32_t getTarget();
	uint32_t getTimeOut();
	dir getStatusDirect();
	statusMotor getStatusRotation();
	uint16_t getRPM();
	mode_rotation_t getMode();
	motor_t getMotor();
	statusTarget_t getStatusTarget();
	uint32_t getLastDistance();

	//methods for aktion
	bool start();
	bool startForCall(dir d);
	void stop(statusTarget_t status); // принимает статус окончания если прошло без ошибок передаем 1
	void slowdown();
	void removeBreak(bool status);
	void goTo(int steps, dir direct);
	void Init();
	bool Calibration_pool();
	void findHome(); // функция поиска домашней позиции
	void CallStart();
	void findHomeStart();


	//handlers
	void StepsHandler(uint32_t steps);
	void StepsAllHandler(uint32_t steps);
	void SensHandler(uint16_t GPIO_Pin);
	void AccelHandler();

    void StartDebounceTimer(uint16_t GPIO_Pin); // запуск таймера антидребезга
    void HandleDebounceTimeout(); // обработчик таймаута

private:
	void InitTim();
	double map(double x, double in_min, double in_max, double out_min, double out_max);

	settings_t *settings{}; // указатель на структуру с настройками
	TIM_HandleTypeDef *TimCountAllSteps;
	TIM_HandleTypeDef *TimFrequencies;
	uint32_t ChannelClock;
	//TIM_HandleTypeDef *TimAcceleration;
	TIM_HandleTypeDef* debounceTimer; // указатель на таймер для антидребезга
	TIM_HandleTypeDef *TimEncoder;
    const uint32_t DEBOUNCE_TIMEOUT = 100; // задержка антидребезга
    volatile bool d0_debounce_active = false;
    volatile bool d1_debounce_active = false;


	//mode_rotation_t mod_rotation = by_meter_enc;

	uint32_t    MaxSpeed = 1;
	uint32_t    MinSpeed = 20000;
	//uint32_t    StartSpeed = MinSpeed;
	//dir			CurentDir = dir::END_OF_LIST;
	uint32_t	Time = 0; 						// отсчет времении до остановки
	uint8_t		TimerIsStart = false;			// статус таймера остановки
	uint32_t	PrevCounterENC = 0;				// хранит предыдущее положение энкодера
	//uint32_t	PrevENC = 0;					// предыдущее состояние энкодера
	uint8_t		countErrDir = 3;				// счетчик ошибой, зашита от дребезга мотора
	uint32_t	CallSteps = 0;					// откалиброванное растояние в шагах между датчиками
	//uint32_t    Accel = 0; 					// ускарение динамически подстраивается под скорость(в отсчетах таймера)
	//uint32_t 	Slowdown;						// торможение в отсчетах таймеры
	uint32_t 	LastDistance = 0;				// расстояние в шагах пройденное за предыдущее действие
	uint32_t	motionSteps = 0;				// шаги за разгон и движение
	uint32_t    Speed_Call = 0; 				// скорость при калибровке
	uint32_t    Speed_temp = 0; 				// временно хранит заданную скорость
	statusMotor Status = statusMotor::STOPPED;
	statusTarget_t StatusTarget = statusTarget_t::finished;
	fb FeedbackType = fb::NON; 					// тип обратной связи

    const uint32_t START_VIBRATION_TIMEOUT = 400; // таймаут в мс для игнорирования вибраций при старте
    bool ignore_sensors = false; // флаг игнорирования датчиков
    uint32_t vibration_start_time = 0; // время начала игнорирования

	pos_t position = pos_t::D_0_1; 		// содержит текущее положение планки
	pos_t target = pos_t::D_0_1; 			// заданная цель (комманда куда ехать)
	uint32_t watchdog = 10000; 					// максимальное время выполнения операции а милисек
	//bool needCall = true; 						// необходимость калибровки
	bool permission_calibrate = false; 			// разрешение на калибровку
	bool permission_findHome = false;
	bool change_pos = false; 					// изменить позицию
	uint32_t time = 0;
	uint8_t bos_bit = 0;
	//uint16_t last_triggered_sensor; // Хранит GPIO_Pin последнего сработавшего датчика
};

