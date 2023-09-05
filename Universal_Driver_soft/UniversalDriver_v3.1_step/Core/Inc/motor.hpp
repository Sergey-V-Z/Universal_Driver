#include "main.h"
#include "cmsis_os.h"
#include "Delay_us_DWT.h"

enum class dir{CW, CCW, END_OF_LIST};
enum class step{HALF, FULL, END_OF_LIST};
enum class statusMotor{MOTION, STOPPED, ACCEL, BRAKING, END_OF_LIST};
enum class fb{ENCODER, HALLSENSOR, NON};
enum class sensorsERROR{OK, No_Connect, No_Sigal};
enum class stepperMode{Stepper, bldc, n};
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
// CLASS: base_motor
//
// DESCRIPTION:
//  --
//
// CREATED: 20.09.2020, by Ierixon-HP
//
// FILE: motor.h
//
class base_motor{
public:
	base_motor(){};
	//base_motor(dir direction, step stepmode, unsigned int accel);
	~base_motor(){};

	//methods for set
	virtual void SetDirection(dir direction) = 0;
	virtual void SetStepMode(step stepmode) = 0;
	virtual void SetAcceleration(uint16_t accel) = 0;
	virtual void SetDeacceleration(uint16_t accel) = 0;
	virtual void SetCurrentMax(unsigned int current) = 0;
	virtual void SetCurrentStop(unsigned int current) = 0;
	virtual void SetSpeed(uint16_t percent) = 0;
	virtual void SetPWRstatus(bool low) = 0;
	virtual void SetTarget (uint32_t Target) = 0;
	virtual void SetZeroPoint (void) = 0;
	virtual void SetMode(bool mod) = 0;
	//virtual void setCurrent(uint32_t mAmax);
	virtual void SetPWM_Mode(uint32_t mode) = 0;

	//methods for get
	virtual uint32_t get_pos() = 0;
	virtual uint32_t getAcceleration() = 0;
	virtual uint32_t getSpeed() = 0;
	virtual uint32_t getTarget() = 0;
	virtual dir getStatusDirect() = 0;
	virtual statusMotor getStatusRotation() = 0;
	virtual uint16_t getRPM() = 0;
	virtual bool getMode() = 0;

	//methods for aktion
	virtual void goTo(int steps, dir direct) = 0;
	virtual void Init(settings_t settings) = 0;
	virtual void start() = 0;
	virtual void stop() = 0;
	virtual void deceleration() = 0;
	virtual void removeBreak(bool status) = 0;

	//handlers
	virtual void SensHandler() = 0;
	virtual void StepsHandler(uint32_t steps) = 0;
	virtual void AccelHandler() = 0;
	virtual void StepsAllHandler(uint32_t steps) = 0;

	uint32_t zeroPoint = 3;

protected:
	double map(double x, double in_min, double in_max, double out_min, double out_max);

	dir    Direction = dir::CCW;
	step   StepMode = step::HALF;
	//const uint16_t    ConstMaxAccel = 265; // при полушаге
	//const uint16_t    ConstMinAccel = 3000;
	uint32_t    MaxSpeed = 1;
	uint32_t    MinSpeed = 60000;
	uint32_t    Accel = 100;
	uint32_t    Deaccel = 10;// процент от всего пути до начала торможения
	uint32_t    Speed = 0;
	uint32_t    Speed_Call = 0; // скорость при калибровке
	uint32_t    Speed_temp = 0; // временно хранит заданную скорость
	uint32_t CurrenrMax = 0;
	uint32_t CurrenrSTOP = 0;
	statusMotor Status = statusMotor::STOPPED;
	uint16_t PWM = 0;
	uint32_t Position = 0; // позиция по обратной связи в данный момент
	fb FeedbackType = fb::NON; // тип обратной связи
	uint32_t CircleCounts = 1000;    // количество отсчетов на круг у обратной связи
	uint32_t MotorCounts = 0;    // количество отсчетов на круг у мотора
	uint32_t target = 500; // переменная хранит позицию до которой нужно ехать по обратной связи
	uint32_t FeedbackBraking_P0 = 0; //начало торможения в отсчетах
	uint32_t FeedbackBraking_P1 = 0; //начало торможения в отсчетах

	uint32_t PWM_Mode = 2; // оба ключа

private:


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
class extern_driver : public base_motor {
public:
	extern_driver();
	extern_driver(TIM_HandleTypeDef *timCount,
			TIM_HandleTypeDef *timFreq, uint32_t channelFreq, TIM_HandleTypeDef *timAccel);
	~extern_driver();

	//methods for set
	virtual void SetDirection(dir direction);
	virtual void SetStepMode(step stepmode);
	void SetSpeed(uint16_t percent);
	void SetAcceleration(uint16_t percent);
	void SetDeacceleration(uint16_t accel);
	void SetCurrent(uint32_t mAmax);
	void SetPWM_Mode(uint32_t mode);
	void SetCurrentMax(unsigned int current);
	void SetCurrentStop(unsigned int current);
	void SetPWRstatus(bool low);
	void SetTarget (uint32_t Target);
	void SetZeroPoint (void);
	void SetMode(bool mod);
	void Parameter_update(void);
	//methods for get
	uint32_t get_pos();
	uint32_t getAcceleration();
	uint32_t getSpeed();
	uint32_t getTarget();
	dir getStatusDirect();
	statusMotor getStatusRotation();
	uint16_t getRPM();
	bool getMode();

	//methods for aktion
	void start();
	void stop();
	void deceleration();
	void removeBreak(bool status);
	void goTo(int steps, dir direct);
	void Init(settings_t settings);

	//handlers
	void StepsHandler(uint32_t steps);
	void StepsAllHandler(uint32_t steps);
	void SensHandler();
	void AccelHandler();

protected:
	double map(double x, double in_min, double in_max, double out_min, double out_max);

private:
	void InitTim();

	TIM_HandleTypeDef *TimCountAllSteps;
	TIM_HandleTypeDef *TimFrequencies;
	TIM_HandleTypeDef *TimAcceleration;
	uint32_t Channel;
	uint32_t ChannelClock;
	uint32_t StepsAccelBreak = 0;
	uint32_t StepsAll = 0;
	uint32_t StepsPassed = 0;
	uint32_t temp = 0;
	bool lowpwr = true;
	bool modCounter = true;

	const uint16_t    ConstMaxAccel_LOWPWR = 355; // при полушаге
	const uint16_t    ConstMinAccel_LOWPWR = 1500;


	//  uint32_t motion = 0;
	//  uint32_t stop = 0;
	stepperMode ModeStepper = stepperMode :: bldc;
	uint32_t HoldingCurrent = 64;



};


/*
//@@@@@@@@@@@@@@@@@@@@@@@@@
// CLASS: BLDC_motor
//
// DESCRIPTION:
//  BLDC motor driver
//
// CREATED: 20.09.2020, by Ierixon-HP
//
// FILE: motor.h
//
class BLDC_motor : public base_motor {
 public:
  BLDC_motor(TIM_HandleTypeDef *tim_1, TIM_HandleTypeDef *tim_2, TIM_HandleTypeDef *xorTim, TIM_HandleTypeDef *encTim);
  BLDC_motor(TIM_HandleTypeDef *tim_1, TIM_HandleTypeDef *tim_2, TIM_HandleTypeDef *xorTim, TIM_HandleTypeDef *encTim, dir direction, step stepmode, unsigned int accel);
  ~BLDC_motor();

  //methods for set
  void SetSpeed(uint16_t percent);
  void SetCurrent(uint32_t mAmax);
  void SetPWM_Mode(uint32_t mode);

  //methods for get
  uint32_t get_pos();
  dir getStatusDirect();
  statusMotor getStatusRotation();
  uint16_t getRPM();

  //methods for aktion
  void start();
  void stop();
  void deceleration();
  void removeBreak(bool status);
  void goTo(int steps, dir direct);
  void Init(settings_t settings);


  //handlers
  void SensHandler();
  void StepsHandler(int steps);
  void AccelHandler();
  void StepsAllHandler(int steps);

 private:

  int currentTimeTurn = 0;
  int counterSteps = 0;
  int RPM = 0;
  float timOneTick = 0;
  uint32_t maxPWM = 0;
  uint32_t delayComm = 400;

  //TIM for PWM
  TIM_HandleTypeDef *TIM_1;
  TIM_HandleTypeDef *TIM_2;
  TIM_HandleTypeDef *XorTim;
  TIM_HandleTypeDef *EncTim;

  //methods for aktion
  uint32_t PWM_Mode_0();
  uint32_t PWM_Mode_1();
  uint32_t PWM_Mode_2();
};
 */
