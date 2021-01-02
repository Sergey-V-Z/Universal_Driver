#include "motor.hpp"

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

void base_motor::SetAcceleration(unsigned int accel){
   if(accel <= 100){
      Acceleration = accel;
   }
}

double base_motor::map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//void base_motor::goTo(int steps, dir direct){
//  
//}


