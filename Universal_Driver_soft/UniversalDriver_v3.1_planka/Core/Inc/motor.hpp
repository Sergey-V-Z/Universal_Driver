//#include <stm32f4xx_hal_dac.h>
//#include <stm32f4xx_hal_tim.h>
//#include <sys/_stdint.h>
#include "main.h"
#include "cmsis_os.h"
#include "Delay_us_DWT.h"

enum class dir{CW, CCW, END_OF_LIST};
enum class step{HALF, FULL, END_OF_LIST};
enum class statusMotor{MOTION, STOPPED, ACCEL, BRAKING, END_OF_LIST};
enum class fb{ENCODER, HALLSENSOR, NON};
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
	base_motor();
	base_motor(dir direction, step stepmode, unsigned int accel);
	~base_motor();

	//methods for set
	void SetDirection(dir direction);
	void SetStepMode(step stepmode);
	virtual void SetAcceleration(uint16_t accel);
	virtual void SetDeacceleration(uint16_t accel);
	virtual void SetCurrentMax(unsigned int current);
	virtual void SetCurrentStop(unsigned int current);
	virtual void SetSpeed(uint16_t percent);
	virtual void SetPWRstatus(bool low);
	virtual void SetFeedbackTarget (uint32_t Target);
	virtual void SetZeroPoint (void);
	//virtual void setCurrent(uint32_t mAmax);
	virtual void SetPWM_Mode(uint32_t mode);

	//methods for get
	virtual uint32_t get_pos();
	virtual dir getStatusDirect();
	virtual statusMotor getStatusRotation();
	virtual uint16_t getRPM();

	//methods for aktion
	virtual void goTo(int steps, dir direct)=0;
	virtual void Init(settings_t settings);
	virtual void start();
	virtual void stop();
	virtual void deceleration();
	virtual void removeBreak(bool status);

	//handlers
	virtual void SensHandler(uint16_t GPIO_Pin);
	virtual void StepsHandler(uint32_t steps);
	virtual void AccelHandler();
	virtual void StepsAllHandler(uint32_t steps);


	const uint16_t    MaxSpeedConst = 1200; // при полушаге
	const uint16_t    MinSpeedConst = 12000;
	uint16_t    MaxSpeed = 1200; // при полушаге
	uint16_t    MinSpeed = 12000;

protected:
	double map(double x, double in_min, double in_max, double out_min, double out_max);

	dir    Direction = dir::CW;
	step   StepMode = step::HALF;
	uint32_t    Acceleration = 30;
	uint16_t    TimeAccelStep = 3000 ; //(1Mhz/timeAccelStep+1 = time)
	uint32_t CurrenrMax = 200;
	statusMotor Status = statusMotor::STOPPED;
	uint16_t PWM = 215;
	uint32_t Position = 0; // позиция по обратной связи в данный момент
	fb FeedbackType = fb::NON; // тип обратной связи
	uint32_t CircleCounts = 0;    // количество отсчетов на круг у обратной связи
	uint32_t MotorCounts = 0;    // количество отсчетов на круг у мотора
	uint32_t FeedbackTarget = 0; // переменная хранит позицию до которой нужно ехать по обратной связи
	uint32_t    Deaccel = 10;// процент от всего пути до начала торможения
	uint32_t    Speed = 0;
	uint32_t    Speed_Call = 0; // скорость при калибровке
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
// FILE: step_motor.h
//
class step_motor : public base_motor {
public:
	step_motor();
	step_motor(DAC_HandleTypeDef *dac, uint32_t channel, TIM_HandleTypeDef *timCount,
			TIM_HandleTypeDef *timCount2, TIM_HandleTypeDef *timFreq, uint32_t channelFreq, TIM_HandleTypeDef *timAccel);
	~step_motor();

	//methods for set
	void setSpeed(uint16_t percent);
	void setCurrent(uint32_t mAmax);
	void SetAcceleration(uint16_t accel);
	void SetDeacceleration(uint16_t deaccel);
	void SetDirection(dir direction);

	//methods for get
	uint32_t get_pos();
	dir getStatusDirect();
	statusMotor getStatusRotation();
	uint16_t getRPM();
	uint32_t getStepsPassed();

	//methods for aktion
	void start();
	void stop();
	void deceleration();
	void removeBreak(bool status);
	void goTo(int steps, dir direct);
	void goTo_twoSteps(int firstSteps, dir direct, uint32_t firstSpeed, uint32_t afterSpeed);
	void Init(settings_t settings);

	//handlers
	void StepsHandler(int steps);
	void StepsAllHandler(int steps);
	void SensHandler(uint16_t GPIO_Pin);
	void AccelHandler();

	TIM_HandleTypeDef *TimCountSteps;
	TIM_HandleTypeDef *TimCountAllSteps;
	TIM_HandleTypeDef *TimFrequencies;
	TIM_HandleTypeDef *TimAcceleration;

private:

	// for two  steps mode
	uint8_t twoStepsMode = 0;
	uint32_t afterSpeed;
	uint32_t stepsPassed = 0;
	uint32_t firstAcceleration = 50; // ускорение для быстрого передвижения
	uint32_t accel = 0;

	DAC_HandleTypeDef *Dac;

	uint32_t Channel;
	uint32_t ChannelClock = TIM_CHANNEL_4;
	uint32_t StepsAccelBreak = 0;
	uint32_t HoldingCurrent = 64;

};

