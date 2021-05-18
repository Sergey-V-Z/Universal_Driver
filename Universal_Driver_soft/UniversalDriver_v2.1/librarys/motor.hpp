#include "main.h"
#include "cmsis_os.h"
#include "Delay_us_DWT.h"

enum class dir{CW, CCW, END_OF_LIST};
enum class step{HALF, FULL, END_OF_LIST};
enum class statusMotor{MOTION, STOPPED, ACCEL, BRAKING, END_OF_LIST};
enum class fb{ENCODER, HALLSENSOR, NON};
enum class sensorsERROR{OK, No_Connect, No_Sigal};
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
  virtual void setSpeed(uint16_t percent);
  virtual void setCurrent(uint32_t mAmax);
  virtual void setPWM_Mode(uint32_t mode);
  
  //methods for get
  virtual uint32_t get_pos();
  virtual dir getStatusDirect();
  virtual statusMotor getStatusRotation();
  virtual uint16_t getRPM();
  
  //methods for aktion
  virtual void goTo(int steps, dir direct)=0;
  virtual void Init();
  virtual void start();
  virtual void stop();
  virtual void deceleration();
  virtual void removeBreak(bool status);
  
  //handlers
  virtual void SensHandler();
  virtual void StepsHandler(int steps);
  virtual void AccelHandler();
  virtual void StepsAllHandler(int steps);
  
 protected:  
  double map(double x, double in_min, double in_max, double out_min, double out_max);
  
  dir    Direction = dir::CW;
  step   StepMode = step::HALF;
  uint32_t    Acceleration = 20; 
  uint16_t    MaxAccel = 110; // при полушаге
  uint16_t    MinAccel = 1000;
  uint16_t    TimeAccelStep = 3000 ; //(1Mhz/timeAccelStep+1 = time)
  uint32_t CurrenrMax = 200;
  statusMotor Status = statusMotor::STOPPED;
  uint32_t Position = 0; // позиция по обратной связи в данный момент
  fb FeedbackType = fb::NON; // тип обратной связи
  uint32_t CircleCounts = 0;    // количество отсчетов на круг у обратной связи
  uint32_t MotorCounts = 0;    // количество отсчетов на круг у мотора
  uint32_t FeedbackTarget = 0; // переменная хранит позицию до которой нужно ехать по обратной связи
    
  uint32_t PWM_Mode = 1; // оба ключа
  
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
  void setSpeed(uint16_t percent);
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
  void setSpeed(uint16_t percent);
  void setCurrent(uint32_t mAmax);
  void setPWM_Mode(uint32_t mode);
  
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
  void StepsAllHandler(int steps);
  void SensHandler();
  void AccelHandler();
  
 private:

  DAC_HandleTypeDef *Dac;
  TIM_HandleTypeDef *TimCountSteps;
  TIM_HandleTypeDef *TimCountAllSteps;
  TIM_HandleTypeDef *TimFrequencies;
  TIM_HandleTypeDef *TimAcceleration;
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
  BLDC_motor(TIM_HandleTypeDef *tim_1, TIM_HandleTypeDef *tim_2, TIM_HandleTypeDef *xorTim, TIM_HandleTypeDef *encTim);
  BLDC_motor(TIM_HandleTypeDef *tim_1, TIM_HandleTypeDef *tim_2, TIM_HandleTypeDef *xorTim, TIM_HandleTypeDef *encTim, dir direction, step stepmode, unsigned int accel);
  ~BLDC_motor();
  
  //methods for set
  void setSpeed(uint16_t percent);
  void setCurrent(uint32_t mAmax);
  void setPWM_Mode(uint32_t mode);

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
  void BLDC_Delay(uint32_t tacts);

  
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
  uint32_t minPWM = 0;
  uint16_t PWM = 215; 
  
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

