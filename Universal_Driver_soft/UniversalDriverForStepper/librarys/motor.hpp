#include "main.h"
#include "cmsis_os.h"


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
  void SetAcceleration(unsigned int accel);
  virtual void setSpeed(uint8_t percent);
  virtual void setCurrent(uint32_t mAmax);
  
  //methods for get
  virtual uint32_t get_pos();
  virtual dir getStatusDirect();
  virtual statusMotor getStatusRotation();
  virtual uint16_t getRPM();
  virtual uint32_t getStepsPassed();
  
  //methods for aktion
  virtual void goTo(int steps, dir direct)=0;
  virtual void goTo_twoSteps(int firstSteps, dir direct, uint32_t firstSpeed, uint32_t afterSpeed);
  virtual void Init();
  virtual void start();
  virtual void stop();
  virtual void deceleration();
  virtual void removeBreak(bool status);
  
  //handlers
  virtual void SensHandler();
  virtual void StepsHandler(int steps);
  
  
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
 private:
  
  
};
//******************
// CLASS: stp_motor
//
// DESCRIPTION:
//  stepper 3 phase motor driver
//
// CREATED: 20.09.2020, by Ierixon-HP
//
//
class step3ph_motor : public base_motor {
 public:
  step3ph_motor();
  step3ph_motor(dir direction, step stepmode, unsigned int accel, DAC_HandleTypeDef *dac, uint32_t channel,
             TIM_HandleTypeDef *timCount, TIM_HandleTypeDef *timFreq, TIM_HandleTypeDef *timAccel);
  ~step3ph_motor();
  
  //methods for set
  void setSpeed(uint8_t percent);
  void setCurrent(uint32_t mAmax);
  
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
  void Init();
  
  //handlers
  void StepsHandler(int steps);
  void SensHandler();
  void AccelHandler();
  
 private:
  enum class motion{accel, rotation, breaking, stopped, END_OF_LIST};

  DAC_HandleTypeDef *Dac;
  TIM_HandleTypeDef *TimCountSteps;
  TIM_HandleTypeDef *TimFrequencies;
  TIM_HandleTypeDef *TimAcceleration;
  uint32_t Channel;
  
  motion MotonStatus;

  
  
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
  void setSpeed(uint8_t percent);
  void setCurrent(uint32_t mAmax);
  
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
  void Init();
  
  //handlers
  void StepsHandler(int steps);
  void StepsAllHandler(int steps);
  void SensHandler();
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


//******************
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
  BLDC_motor(TIM_HandleTypeDef *TIM);
  BLDC_motor(TIM_HandleTypeDef *TIM, dir direction, step stepmode, unsigned int accel);
  ~BLDC_motor();
  
  //methods for set
  void setSpeed(uint8_t percent);
  void setCurrent(uint32_t mAmax);
  
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
  void Init();
  
  //handlers
  void SensHandler();
  
 private:
  
  float RPM = 0;
  uint32_t oldTick = 0;
  //TIM for PWM
  TIM_HandleTypeDef *TIM;
};

