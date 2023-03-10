#include "main.h"
#include "cmsis_os.h"
#include "Delay_us_DWT.h"
#include "stdlib.h"

//enum class dir{CW, CCW, END_OF_LIST};
typedef enum{HALF, FULL}step;
typedef enum{inProgress, finished}statusTarget_t;
typedef enum{MOTION, STOPPED, ACCEL, BRAKING}statusMotor;
typedef enum{ENCODER, HALLSENSOR, NON}fb;
typedef enum{OK, No_Connect, No_Signal}sensorsERROR;
//enum class stepperMode{Stepper, bldc, n};
/*// Special behavior for ++dir
dir& operator++( dir &c ) {
using IntType = typename std::underlying_type<dir>::type;
c = static_cast<dir>( static_cast<IntType>(c) + 1 );
if ( c == dir::END_OF_LIST )
c = static_cast<dir>(0);
return c;
}

// Special behavior for dir++
dir operator++( dir &c, int ) {
dir result = c;
++c;
return result;
}
 */


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
	void SetDeacceleration(uint16_t accel);
	uint32_t SetTarget (uint32_t Target);
	void SetZeroPoint (void);
	void SetMode(bool mod);
	void Parameter_update(void);

	//methods for get
	uint32_t get_pos();
	uint32_t getAccelerationPer();
	uint32_t getSlowdownPer();
	uint32_t getSpeed();
	uint32_t getTarget();
	dir getStatusDirect();
	statusMotor getStatusRotation();
	uint16_t getRPM();
	bool getMode();
	bool getStatusTarget();

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

	//uint32_t Channel;

	uint32_t StepsAccelBreak = 0;
	uint32_t StepsAll = 0;
	uint32_t StepsPassed = 0;
	uint32_t temp = 0;
	//bool lowpwr = true;
	bool modCounter = true;

	//const uint16_t    ConstMaxAccel_LOWPWR = 355; // при полушаге
	//const uint16_t    ConstMinAccel_LOWPWR = 1500;


	//  uint32_t motion = 0;
	//  uint32_t stop = 0;
	//stepperMode ModeStepper = stepperMode :: bldc;
	//uint32_t HoldingCurrent = 64;

	settings_t *settings;			// указатель на структуру с настройками



	//dir    Direction = dir::CCW;
	step   StepMode = step::HALF;
	uint32_t    MaxSpeed = 1;
	uint32_t    MinSpeed = 20000;
	uint32_t    Accel = 0; // ускарение динамически подстраивается под скорость(в отсчетах таймера)
	uint32_t 	Slowdown;			// торможение в отсчетах таймеры
	uint32_t 	SlowdownDistance;	// расстояние для торможения в шагах от всего пути
	//uint32_t    Speed = 0;
	uint32_t    Speed_Call = 0; // скорость при калибровке
	uint32_t    Speed_temp = 0; // временно хранит заданную скорость
	uint32_t CurrenrMax = 0;
	uint32_t CurrenrSTOP = 0;
	statusMotor Status = statusMotor::STOPPED;
	statusTarget_t StatusTarget = statusTarget_t::finished;
	//uint16_t PWM = 0;
	//uint32_t Position = 0; // позиция по обратной связи в данный момент
	fb FeedbackType = fb::NON; // тип обратной связи
	//uint32_t CircleCounts = 1000;    // количество отсчетов на круг у обратной связи
	//uint32_t MotorCounts = 0;    // количество отсчетов на круг у мотора
	//uint32_t target = 500; // переменная хранит позицию до которой нужно ехать по обратной связи
	//uint32_t FeedbackBraking_P0 = 0; //начало торможения в отсчетах
	//uint32_t FeedbackBraking_P1 = 0; //начало торможения в отсчетах
	//uint32_t prevCounter = 65535/2; //для хранения предыдущего состояния энкодера

};

