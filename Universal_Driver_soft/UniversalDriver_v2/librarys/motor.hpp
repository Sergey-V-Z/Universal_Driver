#include "main.h"
#include "cmsis_os.h"


enum class dir{CW, CCW, END_OF_LIST};
enum class step{HALF, FULL, END_OF_LIST};

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
  virtual bool getStatusRotation();
  virtual uint16_t getRPM();
  
  //methods for aktion
  virtual void goTo(int steps, dir direct)=0;
  virtual void Init();
  virtual void start();
  virtual void stop();
  virtual void removeBreak(bool status);
  
  //handlers
  virtual void SensHandler();
  virtual void endMotionHandler(int steps);
  
 protected:  
  double map(double x, double in_min, double in_max, double out_min, double out_max);
  
  dir    Direction = dir::CW;
  step   StepMode = step::HALF;
  uint32_t    Acceleration = 20; 
  uint32_t CurrenrMax = 600;
  bool StartMotor = false;
  uint16_t pwm = 215; 
  uint8_t position = 0;
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
  step_motor(dir direction, step stepmode, unsigned int accel, DAC_HandleTypeDef *dac, uint32_t channel,
             TIM_HandleTypeDef *timCount, TIM_HandleTypeDef *timFreq, TIM_HandleTypeDef *timAccel);
  ~step_motor();
  
  //methods for set
  void setSpeed(uint8_t percent);
  void setCurrent(uint32_t mAmax);
  
  //methods for get
  uint32_t get_pos();
  dir getStatusDirect();
  bool getStatusRotation();
  uint16_t getRPM();
  
  //methods for aktion
  void start();
  void stop();
  void removeBreak(bool status);
  void goTo(int steps, dir direct);
  void Init();
  
  //handlers
  void endMotionHandler(int steps);
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
  uint16_t maxV = 4000; // Hz
  uint16_t minV = 1; // Hz
  
  
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
  bool getStatusRotation();
  uint16_t getRPM();
  
  //methods for aktion
  void start();
  void stop();
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

