#include "motor.hpp"

/***************************************************************************
* Класс для шагового двухфазного мотора
*
* В этом классе реализован цикл управления и контроля шагового двигателя
****************************************************************************/
//methods for set************************************************
void extern_driver::SetSpeed(uint16_t percent){
   if(percent >1000){percent = 1000;}
   if(lowpwr){
     MaxAccel = (uint16_t) map(percent, 0, 1000, ConstMinAccel_LOWPWR, ConstMaxAccel_LOWPWR);
   }else{
     MaxAccel = (uint16_t) map(percent, 0, 1000, ConstMinAccel, ConstMaxAccel);
   }
   
}

void extern_driver::SetCurrent(uint32_t mAmax){
  CurrenrMax = mAmax;
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

   HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, CurrenrMax);
   TimFrequencies->Instance->CNT = 0;
   TimFrequencies->Instance->ARR = MinAccel;
   TimAcceleration->Instance->CNT = 0;
   TimAcceleration->Instance->ARR = TimeAccelStep;
   ModeStepper = stepperMode::bldc;
   
   Status = statusMotor::ACCEL;
   
   HAL_TIM_OC_Start(TimFrequencies, ChannelClock); // старт мотора
   HAL_TIM_Base_Start_IT(TimAcceleration);      //страрт таймера ускарения
  
   
}

void extern_driver::stop(){
 HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
//   removeBreak(false);
}

void extern_driver::deceleration(){
  Status = statusMotor::BRAKING;
  
  HAL_TIM_Base_Start_IT(TimAcceleration);
}

void extern_driver::removeBreak(bool status){
   if(status){
      HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_SET);
   }else{
      HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_RESET);
   }
}

void extern_driver::goTo(int steps, dir direct){
  // из настроек установить параметры системы. количество отсчетов обратной связи и мотора
  
  // если устаровлен режим обратной связи то расчитать позицию для обратной связи
  StepsAll = steps-1;
  StepsPassed = 0;
  TimCountAllSteps->Instance->ARR = steps-1;
  Direction = direct;
  ModeStepper = stepperMode::Stepper;
  if(Direction == dir::CW){
    HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
  }else if(Direction == dir::CCW){
    HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
  }
  
  TimCountAllSteps->Instance->CNT = 0; //обнуляем счетчик шагов
  start();
//  TimFrequencies->Instance->ARR = 1000;
//  HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, 200);
//  HAL_TIM_OC_Start(TimFrequencies, ChannelClock);
   // установить ток для движения
  
  // запустить таймер частоты
}

void extern_driver::Init(){

  //InitTim();
  // init variables
  if(lowpwr){
    MaxAccel = ConstMaxAccel_LOWPWR;
  }else{
    MaxAccel = ConstMaxAccel;
  }
   Status = statusMotor::STOPPED;
   FeedbackType = fb::ENCODER; // сделать установку этого значения из настроек
 
   HAL_DAC_Start(Dac, Channel);
   HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, CurrenrSTOP);
   
   if(Direction == dir::CW){
     HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
   }else if(Direction == dir::CCW){
     HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
   }
   
   if(StepMode == step::HALF){
     HAL_GPIO_WritePin(H_F_GPIO_Port, H_F_Pin, GPIO_PIN_SET);
     TimCountAllSteps->Instance->PSC = 1;
   }else if(StepMode == step::FULL){
     HAL_GPIO_WritePin(H_F_GPIO_Port, H_F_Pin, GPIO_PIN_RESET);
     TimCountAllSteps->Instance->PSC = 3;
   }  
   //TimFrequencies->Instance->PSC = 
   
      // настройка комутации
   HAL_GPIO_WritePin(SW_encDiff_GPIO_Port, SW_encDiff_Pin, GPIO_PIN_RESET);     // reset/set = diff/nodiff
   HAL_GPIO_WritePin(SW_s_h_GPIO_Port, SW_s_h_Pin, GPIO_PIN_SET);             // reset/set = soft/hard(stepper)
   
   // включение преобразователей
   HAL_GPIO_WritePin(EN_hard_GPIO_Port, EN_hard_Pin, GPIO_PIN_SET);     // reset/set = off/on
   HAL_GPIO_WritePin(EN_soft_GPIO_Port, EN_soft_Pin, GPIO_PIN_RESET);     // reset/set = off/on
   
   HAL_GPIO_WritePin(CTRL_GPIO_Port, CTRL_Pin, GPIO_PIN_RESET); // reset/set = PWM in INx/PWM in ABCD
   
   HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
   
   HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET); // enable chip

   HAL_TIM_Base_Start_IT(TimCountAllSteps);
} 


//handlers*******************************************************
//счетчик операционный для счета шагов между операциями
void extern_driver::StepsHandler(int steps){

}

//счетчик обшего количества шагов
void extern_driver::StepsAllHandler(int steps){
  if(Status == statusMotor::MOTION){
    //temp = TimCountAllSteps->Instance->CNT;
    // вычислить сколько осталось шагов до полной остановки
    StepsPassed = StepsPassed + temp;
    //TimCountAllSteps->Instance->CNT = 0; 
    TimCountAllSteps->Instance->ARR = StepsAll - StepsPassed;
    deceleration(); // запуск торможения 
  }
  else if(Status == statusMotor::BRAKING) {
    //если установлен режим обратной связи то проверить позицию по обратной связи и если не дошли то выставить минимальную скорость и в счетчик полный шагов установить 1 и выйти из прерывания
    HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
    HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET); // enable chip
    HAL_TIM_Base_Stop_IT(TimAcceleration);
    HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, CurrenrSTOP);
    Status = statusMotor::STOPPED;
    TimCountAllSteps->Instance->CNT = 0; //обнуляем счетчик шагов
    StepsAll = 0;
    StepsPassed = 0;
    StepsAccelBreak = 0;
    HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET); // enable chip
  }
}

void extern_driver::SensHandler(){
  
}

// обработчик таймера разгона торможения
void extern_driver::AccelHandler(){
  switch(Status)
    {
     case statusMotor::ACCEL:
      {
        // Закончили ускорение
        if(TimFrequencies->Instance->ARR - Acceleration <= MaxAccel){ // если "ускорение" меньше или ровно максимальному то выставить максимум 
          StepsAccelBreak = TimCountAllSteps->Instance->CNT; // сохроняем количестов шагов сделанное при ускорении
          StepsPassed += StepsAccelBreak;
          TimFrequencies->Instance->ARR = MaxAccel;
          Status = statusMotor::MOTION;
          HAL_TIM_Base_Stop_IT(TimAcceleration);
          
          temp = ((StepsAll + 1u) - ((StepsAccelBreak +2u) * 2u)); // расчет шагов до торможения;
          TimCountAllSteps->Instance->ARR = temp;
          
        }else{
          TimFrequencies->Instance->ARR -= Acceleration;
        }
        break;
      }
     case statusMotor::BRAKING:
      {
        if(TimFrequencies->Instance->ARR + Acceleration >= MinAccel){ // если "торможение" больше или ровно минимальному то выставить минимум и остоновить торможение
          TimFrequencies->Instance->ARR = MinAccel;
          
          
          if(ModeStepper == stepperMode :: bldc){
            HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
            HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET); // enable chip
            HAL_TIM_Base_Stop_IT(TimAcceleration);
            HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, CurrenrSTOP);
            Status = statusMotor::STOPPED;
            TimCountAllSteps->Instance->CNT = 0; //обнуляем счетчик шагов
            StepsAll = 0;
            StepsPassed = 0;
            StepsAccelBreak = 0;
            HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET); // enable chip
          }else{
            HAL_TIM_Base_Stop_IT(TimAcceleration);
          }
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
void extern_driver::InitTim(){

}
extern_driver::extern_driver(){
   
}

extern_driver::extern_driver(DAC_HandleTypeDef *dac, uint32_t channel, TIM_HandleTypeDef *timCount, 
                       TIM_HandleTypeDef *timFreq, uint32_t channelFreq , TIM_HandleTypeDef *timAccel) :
 
 Dac(dac), Channel(channel), TimCountAllSteps(timCount), TimFrequencies(timFreq), ChannelClock(channelFreq), TimAcceleration(timAccel){
   
}

