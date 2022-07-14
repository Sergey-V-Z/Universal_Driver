#include "motor.hpp"

/***************************************************************************
* Класс для шагового двухфазного мотора
*
* В этом классе реализован цикл управления и контроля шагового двигателя
****************************************************************************/
//methods for set************************************************
void step_motor::setSpeed(uint16_t percent){
   if(percent >1000){percent = 1000;}
   PWM = (uint16_t) map(percent, 0, 1000, 215, 361);
}

void step_motor::setCurrent(uint32_t mAmax){
   CurrenrMax = mAmax;
}
void step_motor::SetAcceleration(uint16_t percent){
   if(percent >1000){percent = 1000;}
   if(percent <1){percent = 1;}
   Acceleration = (uint16_t) map(percent, 1, 1000, 1, Speed);
}

void step_motor::SetDirection(dir direction){
   Direction = direction;
   if(Direction == dir::CW){
      HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
   }else if(Direction == dir::CCW){
      HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
   }
}
void step_motor::SetDeacceleration(uint16_t deaccel){

}

void step_motor::SetRotationRange(uint32_t Range){
	StepsAccelBreak = Range;
	//пересчитать необходимые переменные


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
uint32_t step_motor::getStepsPassed(){
   return stepsPassed;
}
//methods for aktion*********************************************
void step_motor::start(){
   
   //   removeBreak(true);
   //HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, CurrenrMax);
   TimFrequencies->Instance->CNT = 0;
   TimFrequencies->Instance->ARR = MinSpeed;
   TimAcceleration->Instance->CNT = 0;
   TimAcceleration->Instance->ARR = TimeAccelStep;
   
   Status = statusMotor::ACCEL;
   HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET); // enable chip
   HAL_TIM_OC_Start(TimFrequencies, ChannelClock); // старт мотора
   HAL_TIM_Base_Start_IT(TimAcceleration);      //страрт таймера ускарения
   
   
}

void step_motor::stop(){
   HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
   //HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, 64);
   Status = statusMotor::STOPPED; 
   HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET); // Disable chip
   //   removeBreak(false);
}

void step_motor::deceleration(){
   Status = statusMotor::BRAKING;
   
   HAL_TIM_Base_Start_IT(TimAcceleration);
}

void step_motor::removeBreak(bool status){
   if(status){
      //HAL_GPIO_WritePin(Brake_GPIO_Port, Brake_Pin, GPIO_PIN_SET);
   }else{
      //HAL_GPIO_WritePin(Brake_GPIO_Port, Brake_Pin, GPIO_PIN_RESET);
   }
}

void step_motor::goTo(int steps, dir direct){
   // из настроек установить параметры системы. количество отсчетов обратной связи и мотора
   
   // если устаровлен режим обратной связи то расчитать позицию для обратной связи
   TimCountAllSteps->Instance->ARR = steps-1;

   SetDirection(direct);
   
   TimCountSteps->Instance->CNT = 0; //обнуляем счетчик шагов
   TimCountAllSteps->Instance->CNT = 0; //обнуляем счетчик шагов
   start();
   //  TimFrequencies->Instance->ARR = 1000;
   //  HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, 200);
   //  HAL_TIM_OC_Start(TimFrequencies, ChannelClock);
   // установить ток для движения
   
   // запустить таймер частоты
}

void step_motor::goTo_twoSteps(int firstSteps, dir direct, uint32_t firstSpeed, uint32_t afterSpeed){
   
   twoStepsMode = 1; // режим двух скоростей влючить
   MaxSpeed = firstSpeed; //установка максимальной скорости
   step_motor::afterSpeed = afterSpeed; //сохраняем вторую скорость
   
   TimCountAllSteps->Instance->ARR = firstSteps-1;
   SetDirection(direct);
   
   TimCountSteps->Instance->CNT = 0; //обнуляем счетчик шагов
   TimCountAllSteps->Instance->CNT = 0; //обнуляем счетчик шагов
   start();

}

void step_motor::Init(settings_t settings){
   // init variables
   Status = statusMotor::STOPPED;
   FeedbackType = fb::NON; // сделать установку этого значения из настроек
   
   HAL_DAC_Start(Dac, Channel);
   HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, HoldingCurrent);
   
   SetDirection(Direction);
   
   HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET); // Disable chip
   
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
   
   // если режим двух скоростей
   if(twoStepsMode){
      twoStepsMode = 0; // отключаем режим двух скоростей
      
      // меняем скорость
      TimFrequencies->Instance->CNT = 0;
      TimFrequencies->Instance->ARR = afterSpeed; // установка второй скорости
      
      stepsPassed = TimCountAllSteps->Instance->CNT; //сохроняем количество пройденных шагов
      TimCountAllSteps->Instance->CNT = 0; //обнуляем счетчик шагов
      TimCountAllSteps->Instance->ARR = 20000; // количество шагов на второй скорости

   }
   else{
      if(Status == statusMotor::BRAKING){
         //если установлен режим обратной связи то проверить позицию по 
         //обратной связи и если не дошли то выставить минимальную 
         //скорость и в счетчик полный шагов установить 1 и выйти из прерывания
         
         HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
         HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, 64);
         Status = statusMotor::STOPPED;
         
         stepsPassed += TimCountAllSteps->Instance->CNT; //сохроняем количество пройденных шагов
         TimCountSteps->Instance->CNT = 0; //обнуляем счетчик шагов
         TimCountAllSteps->Instance->CNT = 0; //обнуляем счетчик шагов
      }
   }
}

void step_motor::SensHandler(uint16_t GPIO_Pin){
   
}

// обработчик таймера разгона торможения
void step_motor::AccelHandler(){
   
   if(twoStepsMode){
      accel = firstAcceleration;
   }
   else{
      accel = Acceleration;      
   }
   switch(Status)
   {
     case statusMotor::ACCEL:
      {
         
         if(TimFrequencies->Instance->ARR - accel <= MaxSpeed){ // если "ускорение" меньше или ровно максимальному то выставить максимум 
            StepsAccelBreak = TimCountAllSteps->Instance->CNT; // сохроняем количестов шагов сделанное при ускорении
            
            TimCountSteps->Instance->CNT = 0;
            TimFrequencies->Instance->ARR = MaxSpeed;
            Status = statusMotor::MOTION;
            HAL_TIM_Base_Stop_IT(TimAcceleration);
            
            TimCountSteps->Instance->ARR = ((TimCountAllSteps->Instance->ARR + 1) - ((StepsAccelBreak +2) * 2)); // расчет шагов до торможения
         }else{
            TimFrequencies->Instance->ARR -= accel;
         }
         break;
      }
     case statusMotor::BRAKING:
      {
         if(TimFrequencies->Instance->ARR + accel >= MinSpeed){ // если "торможение" больше или ровно минимальному то выставить минимум и остоновить торможение
            TimFrequencies->Instance->ARR = MinSpeed;
            
            HAL_TIM_Base_Stop_IT(TimAcceleration);
            //HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
            //HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, 64);
         }else{
            TimFrequencies->Instance->ARR += accel;
         }
         break;
      }
     default:
      {
         break;
      }
      
   }
   
   
}

void RefreshVar(){

}
//*******************************************************
step_motor::step_motor(){
   
}

step_motor::step_motor(DAC_HandleTypeDef *dac, uint32_t channel, TIM_HandleTypeDef *timCount, TIM_HandleTypeDef *timCount2, TIM_HandleTypeDef *timFreq,uint32_t channelFreq , TIM_HandleTypeDef *timAccel) :
 
 Dac(dac), Channel(channel), TimCountAllSteps(timCount2), TimCountSteps(timCount), TimFrequencies(timFreq), ChannelClock(channelFreq), TimAcceleration(timAccel){
    
}
 
step_motor::~step_motor(){}

