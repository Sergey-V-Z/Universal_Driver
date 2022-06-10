#include <motor.hpp>

/***************************************************************************
* Базовый класс для моторов
*
*
****************************************************************************/
base_motor::base_motor(){}

base_motor::base_motor(dir direction, step stepmode, unsigned int accel){
   Direction = direction;
   StepMode = stepmode;
   if(accel <= 100){
      Acceleration = accel;
   }else{
      Acceleration = 100;
   }
   
}

void base_motor::SetDirection(dir direction){
   Direction = direction;
}

void base_motor::SetStepMode(step stepmode){
   StepMode = stepmode;
}

void base_motor::SetAcceleration(uint16_t accel){
   if(accel <= 100){
      Acceleration = accel;
   }
}
base_motor::~base_motor(){

}
double base_motor::map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void base_motor::goTo(int steps, dir direct){

}

//methods for set
   void base_motor::SetCurrentMax(unsigned int current){}
   void base_motor::SetCurrentStop(unsigned int current){}
   void base_motor::SetSpeed(uint16_t percent){}
   void base_motor::SetPWRstatus(bool low){}
  //virtual void setCurrent(uint32_t mAmax);
   void base_motor::SetPWM_Mode(uint32_t mode){}

  //methods for get
   uint32_t base_motor::get_pos(){return -1;}
   dir base_motor::getStatusDirect(){return dir::END_OF_LIST;}
   statusMotor base_motor::getStatusRotation(){return statusMotor::END_OF_LIST;}
   uint16_t base_motor::getRPM(){return -1;}

  //methods for aktion
   void base_motor::Init(settings_t settings){}
   void base_motor::start(){}
   void base_motor::stop(){}
   void base_motor::deceleration(){}
   void base_motor::removeBreak(bool status){}

  //handlers
   void base_motor::SensHandler(){}
   void base_motor::StepsHandler(int steps){}
   void base_motor::AccelHandler(){}
void base_motor::StepsAllHandler(int steps) {
}

void base_motor::SetZeroPoint(void) {
}

void base_motor::SetDeacceleration(uint16_t accel) {
}

void base_motor::SetFeedbackTarget(uint32_t Target) {
}
