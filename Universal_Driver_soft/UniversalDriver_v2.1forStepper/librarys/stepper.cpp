#include "motor.hpp"

/***************************************************************************
* Класс для шагового двухфазного мотора
*
* В этом классе реализован цикл управления и контроля шагового двигателя
****************************************************************************/
//methods for set************************************************
void step_motor::SetSpeed(uint8_t percent){
   if(percent >100){percent = 100;}
   if(lowpwr){
     MaxAccel = (uint16_t) map(percent, 0, 100, ConstMinAccel_LOWPWR, ConstMaxAccel_LOWPWR);
   }else{
     MaxAccel = (uint16_t) map(percent, 0, 100, ConstMinAccel, ConstMaxAccel);
   }
   
}

void step_motor::SetCurrent(uint32_t mAmax){
  CurrenrMax = mAmax;
}

void step_motor::SetPWM_Mode(uint32_t mod){
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
void step_motor::SetCurrentMax(unsigned int current){
  if(current < CurrenrSTOP){
    CurrenrMax = CurrenrSTOP;
  }else{
    CurrenrMax = current;
  }
  
}
void step_motor::SetCurrentStop(unsigned int current){
  CurrenrSTOP = current;
}
void step_motor::SetPWRstatus(bool low){
  lowpwr = low;
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
   TimFrequencies->Instance->ARR = MinAccel;
   TimAcceleration->Instance->CNT = 0;
   TimAcceleration->Instance->ARR = TimeAccelStep;
   
   Status = statusMotor::ACCEL;
   
   HAL_TIM_OC_Start(TimFrequencies, ChannelClock); // старт мотора
   HAL_TIM_Base_Start_IT(TimAcceleration);      //страрт таймера ускарения
  
   
}

void step_motor::stop(){
 HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
//   removeBreak(false);
}

void step_motor::deceleration(){
  Status = statusMotor::BRAKING;
  
  HAL_TIM_Base_Start_IT(TimAcceleration);
}

void step_motor::removeBreak(bool status){
   if(status){
      HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_SET);
   }else{
      HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_RESET);
   }
}

void step_motor::goTo(int steps, dir direct){
  // из настроек установить параметры системы. количество отсчетов обратной связи и мотора
  
  // если устаровлен режим обратной связи то расчитать позицию для обратной связи
  StepsAll = steps-1;
  StepsPassed = 0;
  TimCountAllSteps->Instance->ARR = steps-1;
  Direction = direct;
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

void step_motor::Init(){

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
   HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, HoldingCurrent);
   
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
   
   HAL_GPIO_WritePin(CTRL_GPIO_Port, CTRL_Pin, GPIO_PIN_SET); // PWM in ABCD
   
   HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
   
   HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET); // enable chip

   HAL_TIM_Base_Start_IT(TimCountAllSteps);
} 


//handlers*******************************************************
//счетчик операционный для счета шагов между операциями
void step_motor::StepsHandler(int steps){

}

//счетчик обшего количества шагов
void step_motor::StepsAllHandler(int steps){
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
    HAL_TIM_Base_Stop_IT(TimAcceleration);
    HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, 64);
    Status = statusMotor::STOPPED;
    TimCountAllSteps->Instance->CNT = 0; //обнуляем счетчик шагов
    StepsAll = 0;
    StepsPassed = 0;
    StepsAccelBreak = 0;
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
            HAL_TIM_Base_Stop_IT(TimAcceleration);
            HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, 64);
            Status = statusMotor::STOPPED;
            TimCountAllSteps->Instance->CNT = 0; //обнуляем счетчик шагов
            StepsAll = 0;
            StepsPassed = 0;
            StepsAccelBreak = 0;
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
void step_motor::InitTim(){
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  TimFrequencies->Init.Prescaler = 71;
  TimFrequencies->Init.CounterMode = TIM_COUNTERMODE_UP;
  TimFrequencies->Init.Period = 799;
  TimFrequencies->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TimFrequencies->Init.RepetitionCounter = 0;
  TimFrequencies->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_OC_Init(TimFrequencies) != HAL_OK)
  {
    Error_Handler();
  }
  switch(ChannelClock)
    {
     case TIM_CHANNEL_1:
      {
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
        break;
      }
     case TIM_CHANNEL_2:
      {
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
        break;
      }
     case TIM_CHANNEL_3:
      {
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC3REF;
        break;
      }
     case TIM_CHANNEL_4:
      {
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
        break;
      }
//     case TIM_CHANNEL_5:
//      {
//        sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC5REF;
//        break;
//      }
//     case TIM_CHANNEL_6:
//      {
//        sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC6REF;
//        break;
//      }
     default:
      {
        Error_Handler();
        break;
      }
      
    }

  //sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(TimFrequencies, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(TimFrequencies, &sConfigOC, ChannelClock) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(TimFrequencies, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

}
step_motor::step_motor(){
   
}

step_motor::step_motor(DAC_HandleTypeDef *dac, uint32_t channel, TIM_HandleTypeDef *timCount, 
                       TIM_HandleTypeDef *timFreq, uint32_t channelFreq , TIM_HandleTypeDef *timAccel) :
 
 Dac(dac), Channel(channel), TimCountAllSteps(timCount), TimFrequencies(timFreq), ChannelClock(channelFreq), TimAcceleration(timAccel){
   
}

