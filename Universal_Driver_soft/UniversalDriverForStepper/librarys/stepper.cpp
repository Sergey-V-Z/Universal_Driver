#include "motor.hpp"

/***************************************************************************
* Класс для шагового двухфазного мотора
*
* В этом классе реализован цикл управления и контроля шагового двигателя
****************************************************************************/
//methods for set************************************************
void step_motor::setSpeed(uint8_t percent){
   if(percent >100){percent = 100;}
   PWM = (uint16_t) map(percent, 0, 100, 215, 361);
}

void step_motor::setCurrent(uint32_t mAmax){
  CurrenrMax = mAmax;
}

//methods for get************************************************
uint32_t step_motor::get_pos(){
 return 0; 
}

dir step_motor::getStatusDirect(){
   return Direction;
}

statusMotor step_motor::getStatusRotation(){
   return Status;
}

uint16_t step_motor::getRPM(){
   return 0;
}

//methods for aktion*********************************************
void step_motor::start(){
   
//   removeBreak(true);
   HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, CurrenrMax);
   TimFrequencies->Instance->CNT = 0;
   TimFrequencies->Instance->ARR = MinSpeed;
   TimAcceleration->Instance->CNT = 0;
   TimAcceleration->Instance->ARR = TimeAccelStep;
   
   Status = statusMotor::ACCEL;
   
   HAL_TIM_OC_Start(TimFrequencies, ChannelClock); // старт мотора
   HAL_TIM_Base_Start_IT(TimAcceleration);      //страрт таймера ускарения
  
   
}

void step_motor::stop(){
  HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
  HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, 64);
  Status = statusMotor::STOPPED; 
  //   removeBreak(false);
}

void step_motor::deceleration(){
  Status = statusMotor::BRAKING;
  
  HAL_TIM_Base_Start_IT(TimAcceleration);
}

void step_motor::removeBreak(bool status){
   if(status){
      HAL_GPIO_WritePin(Brake_GPIO_Port, Brake_Pin, GPIO_PIN_SET);
   }else{
      HAL_GPIO_WritePin(Brake_GPIO_Port, Brake_Pin, GPIO_PIN_RESET);
   }
}

void step_motor::goTo(int steps, dir direct){
  // из настроек установить параметры системы. количество отсчетов обратной связи и мотора
  
  // если устаровлен режим обратной связи то расчитать позицию для обратной связи
  TimCountAllSteps->Instance->ARR = steps-1;
  Direction = direct;
  if(Direction == dir::CW){
    HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
  }else if(Direction == dir::CCW){
    HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
  }
  
  TimCountSteps->Instance->CNT = 0; //обнуляем счетчик шагов
  TimCountAllSteps->Instance->CNT = 0; //обнуляем счетчик шагов
  start();
//  TimFrequencies->Instance->ARR = 1000;
//  HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, 200);
//  HAL_TIM_OC_Start(TimFrequencies, ChannelClock);
   // установить ток для движения
  
  // запустить таймер частоты
}

void step_motor::Init(){
   // init variables
   Status = statusMotor::STOPPED;
   FeedbackType = fb::ENCODER; // сделать установку этого значения из настроек
 
   HAL_DAC_Start(Dac, Channel);
   HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, HoldingCurrent);
   
   if(Direction == dir::CW){
     HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
   }else if(Direction == dir::CCW){
     HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
   }
   
   if(StepMode == step::HALF){
     HAL_GPIO_WritePin(H_F_GPIO_Port, H_F_Pin, GPIO_PIN_SET);
     TimCountSteps->Instance->PSC = 1;
   }else if(StepMode == step::FULL){
     HAL_GPIO_WritePin(H_F_GPIO_Port, H_F_Pin, GPIO_PIN_RESET);
     TimCountSteps->Instance->PSC = 3;
   }  
   
   HAL_GPIO_WritePin(CTRL_GPIO_Port, CTRL_Pin, GPIO_PIN_SET); // PWM in ABCD
   
   HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
   
   HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET); // enable chip

   HAL_TIM_Base_Start_IT(TimCountSteps); 
   HAL_TIM_Base_Start_IT(TimCountAllSteps);
} 


//handlers*******************************************************
//счетчик операционный для счета шагов между операциями
void step_motor::StepsHandler(int steps){
  if(Status == statusMotor::MOTION){
    deceleration(); 
  }
}

//счетчик обшего количества шагов
void step_motor::StepsAllHandler(int steps){

  if(Status == statusMotor::BRAKING){
    //если установлен режим обратной связи то проверить позицию по 
    //обратной связи и если не дошли то выставить минимальную 
    //скорость и в счетчик полный шагов установить 1 и выйти из прерывания
    
    HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
    HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, 64);
    Status = statusMotor::STOPPED;
    TimCountSteps->Instance->CNT = 0; //обнуляем счетчик шагов
    TimCountAllSteps->Instance->CNT = 0; //обнуляем счетчик шагов
  }
}

void step_motor::SensHandler(){
  
}

// обработчик таймера разгона торможения
void step_motor::AccelHandler(){
  switch(Status)
    {
     case statusMotor::ACCEL:
      {
        
        if(TimFrequencies->Instance->ARR - Acceleration <= MaxSpeed){ // если "ускорение" меньше или ровно максимальному то выставить максимум 
          StepsAccelBreak = TimCountAllSteps->Instance->CNT; // сохроняем количестов шагов сделанное при ускорении
          
//          // жесткий костыль по кокойто причине при первом запуске счетчик считает не верно. возможно я когда нибудь выясню причину
//          if(StepsAccelBreak < 30){
//            TimFrequencies->Instance->ARR -= Acceleration;
//            break;
//          }
          
          TimCountSteps->Instance->CNT = 0;
          TimFrequencies->Instance->ARR = MaxSpeed;
          Status = statusMotor::MOTION;
          HAL_TIM_Base_Stop_IT(TimAcceleration);
          
          TimCountSteps->Instance->ARR = ((TimCountAllSteps->Instance->ARR + 1) - ((StepsAccelBreak +2) * 2)); // расчет шагов до торможения
        }else{
          TimFrequencies->Instance->ARR -= Acceleration;
        }
        break;
      }
     case statusMotor::BRAKING:
      {
        if(TimFrequencies->Instance->ARR + Acceleration >= MinSpeed){ // если "торможение" больше или ровно минимальному то выставить минимум и остоновить торможение
          TimFrequencies->Instance->ARR = MinSpeed;
          
          HAL_TIM_Base_Stop_IT(TimAcceleration);
          //HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
          //HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, 64);
        }else{
          TimFrequencies->Instance->ARR += Acceleration;
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
step_motor::step_motor(){
   
}

step_motor::step_motor(DAC_HandleTypeDef *dac, uint32_t channel, TIM_HandleTypeDef *timCount, 
                       TIM_HandleTypeDef *timCount2, TIM_HandleTypeDef *timFreq,uint32_t channelFreq , TIM_HandleTypeDef *timAccel) :
 
 Dac(dac), Channel(channel), TimCountAllSteps(timCount2), TimCountSteps(timCount), TimFrequencies(timFreq), ChannelClock(channelFreq), TimAcceleration(timAccel){
   
}

