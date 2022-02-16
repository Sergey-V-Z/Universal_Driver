#include "motor.hpp"

/***************************************************************************
* Класс для шагового двухфазного мотора
*
* В этом классе реализован цикл управления и контроля шагового двигателя
****************************************************************************/
//methods for set************************************************
void extern_driver::SetSpeed(uint16_t percent){
   if(percent >1000){percent = 1000;}
   if(percent <1){percent = 1;}
   Speed = (uint16_t) map(percent, 1, 1000, MinSpeed, MaxSpeed);
   if(Status == statusMotor::MOTION){
     TimFrequencies->Instance->CCR1 = Speed;
   }
}

void extern_driver::SetAcceleration(uint16_t percent){
   if(percent >1000){percent = 1000;}
   if(percent <1){percent = 1;}
   Accel = (uint16_t) map(percent, 1, 1000, 1, Speed);
}

void extern_driver::SetDeacceleration(uint16_t percent){
   if(percent >1000){percent = 1000;}
   if(percent <1){percent = 1;}
   Deaccel = (uint16_t) map(percent, 1, 1000, 1, Speed);
}

void extern_driver::SetCurrent(uint32_t mAmax){
  CurrenrMax = mAmax;
}

void extern_driver::SetFeedbackTarget (uint32_t Target){
  if(Target >1000){Target = 1000;}
  if(Target <1){Target = 1;}
  FeedbackTarget = Target;
}

void extern_driver::SetPWM_Mode(uint32_t mod){
  switch(mod)
    {
     case 0: // по нижнему ключу
       {
         //pPWM_Mode_Func  = 
         break;
       }
     case 1: // по верхнему ключу 
      {
        break;
      }
     case 2: // по обоим ключам
      {
        break;
      }
     default: // ошибка
      {
        break;
      }
      
    }

}
void extern_driver::SetCurrentMax(unsigned int current){
  if(current < CurrenrSTOP){
    CurrenrMax = CurrenrSTOP;
  }else{
    CurrenrMax = current;
  }
  
}
void extern_driver::SetCurrentStop(unsigned int current){
  CurrenrSTOP = current;
}
void extern_driver::SetPWRstatus(bool low){
  lowpwr = low;
}
//methods for get************************************************
uint32_t extern_driver::get_pos(){
 return 0; 
}

dir extern_driver::getStatusDirect(){
   return Direction;
}

statusMotor extern_driver::getStatusRotation(){
   return Status;
}

uint16_t extern_driver::getRPM(){
   return 0;
}

//methods for aktion*********************************************
void extern_driver::start(){
   //   removeBreak(true);
   if(Status == statusMotor::STOPPED){
      //Установка направления
      if(Direction == dir::CW){
         HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
      }
      else{
         HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
      }
      
      TimAcceleration->Instance->CCR1 = 0;
      Status = statusMotor::ACCEL;
   }
}

void extern_driver::stop(){
  //   removeBreak(false); 
  if(Status == statusMotor::MOTION){
    (TimFrequencies->Instance->CCR1) = 0;
    Status = statusMotor::STOPPED;
  }
}

void extern_driver::deceleration(){
  if(Status == statusMotor::MOTION){
    Status = statusMotor::BRAKING;
  }
}

void extern_driver::removeBreak(bool status){
   if(status){
      HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_SET);
   }else{
      HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_RESET);
   }
}

void extern_driver::goTo(int steps, dir direct){

}

void extern_driver::Init(settings_t settings){

  //InitTim();
  // init variables

   
   //Расчет максималных параметров PWM для скорости
   MaxSpeed =  ((TimFrequencies->Instance->ARR/100)*90);
   MinSpeed =  ((TimFrequencies->Instance->ARR/100)*5);
   Speed = MinSpeed;
   
   Status = statusMotor::STOPPED;
   FeedbackType = fb::ENCODER; // сделать установку этого значения из настроек
   
   SetAcceleration(settings.Accel); // ускорение
   
   HAL_DAC_Start(Dac, Channel);
   HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, CurrenrSTOP);
   
   if(Direction == dir::CW){
     HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
   }else if(Direction == dir::CCW){
     HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
   }
   
   if(StepMode == step::HALF){
     HAL_GPIO_WritePin(H_F_GPIO_Port, H_F_Pin, GPIO_PIN_SET);
     //TimCountAllSteps->Instance->PSC = 1;
   }else if(StepMode == step::FULL){
     HAL_GPIO_WritePin(H_F_GPIO_Port, H_F_Pin, GPIO_PIN_RESET);
     //TimCountAllSteps->Instance->PSC = 3;
   }  
   //TimFrequencies->Instance->PSC = 
   
   HAL_TIM_Encoder_Start_IT(TimCountAllSteps, TIM_CHANNEL_ALL);
   HAL_TIM_PWM_Start(TimFrequencies, TIM_CHANNEL_1);   
   
   // включение преобразователей
   HAL_GPIO_WritePin(EN_hard_GPIO_Port, EN_hard_Pin, GPIO_PIN_SET);     // reset/set = off/on
   HAL_GPIO_WritePin(EN_soft_GPIO_Port, EN_soft_Pin, GPIO_PIN_RESET);     // reset/set = off/on
   
   // настройка комутации
   HAL_GPIO_WritePin(SW_encDiff_GPIO_Port, SW_encDiff_Pin, GPIO_PIN_RESET);     // reset/set = diff/nodiff
   HAL_GPIO_WritePin(SW_s_h_GPIO_Port, SW_s_h_Pin, GPIO_PIN_SET);             // reset/set = soft/hard(stepper)
   
   HAL_GPIO_WritePin(CTRL_GPIO_Port, CTRL_Pin, GPIO_PIN_RESET); // reset/set = PWM in INx/PWM in ABCD
   
   HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
   
   HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET); // enable chip


} 


//handlers*******************************************************
//счетчик операционный для счета шагов между операциями
void extern_driver::StepsHandler(int steps){

}

//счетчик обшего количества шагов
void extern_driver::StepsAllHandler(int steps){
  //stop();
  // инкрементировать счетчик
  //Position ++;
  // при достижении заданного значения остановка
  if((steps >= FeedbackTarget)&& (Position == 0)){
    deceleration();
    Position = 1;
  } 
}

void extern_driver::SensHandler(){
  // энкодер сделал оборот
  //сбросить счетчик енкодера
  TimCountAllSteps->Instance->CNT = 0;
  Position = 0;
  stop();
  
}

// обработчик таймера разгона торможения
void extern_driver::AccelHandler(){
  switch(Status)
    {
     case statusMotor::ACCEL:
      {
        // Закончили ускорение
        if((TimFrequencies->Instance->CCR1) < Speed){ // если "ускорение" меньше или ровно максимальному то выставить максимум 
          (TimFrequencies->Instance->CCR1) += Accel; // Ускоряем
        }else{        
          (TimFrequencies->Instance->CCR1) = Speed;
          Status = statusMotor::MOTION;
        }
        break;
      }
     case statusMotor::BRAKING:
      {
        if((TimFrequencies->Instance->CCR1) >= MinSpeed){ // если "торможение" больше или ровно минимальному то выставить минимум и остоновить торможение
          (TimFrequencies->Instance->CCR1) -= Deaccel;
        }else{
          (TimFrequencies->Instance->CCR1) = 0;
          Status = statusMotor::STOPPED;
        }
        break;
      }
     default:
      {
        break;
      }
      
    }
  
  
}

//*******************************************************
void extern_driver::InitTim(){

}
extern_driver::extern_driver(){
   
}

extern_driver::extern_driver(DAC_HandleTypeDef *dac, uint32_t channel, TIM_HandleTypeDef *timCount, 
                       TIM_HandleTypeDef *timFreq, uint32_t channelFreq , TIM_HandleTypeDef *timAccel) :
 
 Dac(dac), Channel(channel), TimCountAllSteps(timCount), TimFrequencies(timFreq), ChannelClock(channelFreq), TimAcceleration(timAccel){
   
}

