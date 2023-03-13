#include "main.h"
#include "cmsis_os.h"
#include "Delay_us_DWT.h"
#include "stdlib.h"

//enum class dir{CW, CCW, END_OF_LIST};
typedef enum{HALF, FULL}step;
typedef enum{inProgress = 0, finished, errMotion, errDirection}statusTarget_t;
typedef enum{MOTION, STOPPED, ACCEL, BRAKING}statusMotor;
typedef enum{ENCODER, HALLSENSOR, NON}fb;
typedef enum{OK, No_Connect, No_Signal}sensorsERROR;


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
	extern_driver(TIM_HandleTypeDef *timCount,TIM_HandleTypeDef *timFreq, uint32_t channelFreq, TIM_HandleTypeDef *timAccel, TIM_HandleTypeDef *timENC);
	~extern_driver();

	//methods for set
	void SetDirection(dir direction);
	void SetSpeed(uint16_t percent);
	void SetAcceleration(uint16_t percent);
	void SetSlowdown(uint16_t accel);
	uint32_t SetTarget (uint32_t Target);
	void setTimeOut(uint32_t time);
	void SetZeroPoint (void);
	void SetMode(bool mod);
	void Parameter_update(void);

	//methods for get
	uint32_t get_pos();
	uint32_t getAccelerationPer();
	uint32_t getSlowdownPer();
	uint32_t getSpeed();
	uint32_t getTarget();
	uint32_t getTimeOut();
	dir getStatusDirect();
	statusMotor getStatusRotation();
	uint16_t getRPM();
	bool getMode();
	uint8_t getStatusTarget();

	//methods for aktion
	bool start();
	void stop();
	void slowdown();
	void removeBreak(bool status);
	void goTo(int steps, dir direct);
	void Init(settings_t *settings);

	//handlers
	void StepsHandler(uint32_t steps);
	void StepsAllHandler(uint32_t parent);
	void HandlerBrakingPoint();
	void HandlerStop();
	void SensHandler();
	void AccelHandler();

private:
	void InitTim();
	double map(double x, double in_min, double in_max, double out_min, double out_max);

	TIM_HandleTypeDef *TimCountAllSteps;
	TIM_HandleTypeDef *TimFrequencies;
	uint32_t ChannelClock;
	TIM_HandleTypeDef *TimAcceleration;
	TIM_HandleTypeDef *TimEncoder;

	bool modCounter = true;

	settings_t *settings;						// указатель на структуру с настройками

	uint32_t    MaxSpeed = 1;
	uint32_t    MinSpeed = 20000;
	uint32_t	Time = 0; 						// отсчет времении до остановки
	uint32_t	PrevENC = 0;					// предыдущее состояние энкодера
	uint32_t    Accel = 0; 						// ускарение динамически подстраивается под скорость(в отсчетах таймера)
	uint32_t 	Slowdown;						// торможение в отсчетах таймеры
	uint32_t 	SlowdownDistance;				// расстояние для торможения в шагах от всего пути
	uint32_t    Speed_Call = 0; 				// скорость при калибровке
	uint32_t    Speed_temp = 0; 				// временно хранит заданную скорость
	statusMotor Status = statusMotor::STOPPED;
	statusTarget_t StatusTarget = statusTarget_t::finished;
	fb FeedbackType = fb::NON; // тип обратной связи


};

