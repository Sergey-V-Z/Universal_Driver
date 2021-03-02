#include "motor.hpp"

/***************************************************************************
* Класс для шагового трехфазного мотора
*
* В этом классе реализован цикл управления и контроля шагового двигателя
****************************************************************************/
//methods for set************************************************
void step3ph_motor::SetSpeed(uint8_t percent){
   if(percent >100){percent = 100;}
   PWM = (uint16_t) map(percent, 0, 100, 215, 361);
}

void step3ph_motor::SetCurrent(uint32_t mAmax){
  CurrenrMax = mAmax;
}

//methods for get************************************************
uint32_t step3ph_motor::get_pos(){
 return 0; 
}

dir step3ph_motor::getStatusDirect(){
   return Direction;
}

statusMotor step3ph_motor::getStatusRotation(){
   return Status;
}

uint16_t step3ph_motor::getRPM(){
   return 0;
}

//methods for aktion*********************************************
void step3ph_motor::start(){
   
//   removeBreak(true);
   Status = statusMotor::MOTION;
   
   // Acceleration will be here
   
}

void step3ph_motor::stop(){
   

   Status = statusMotor::STOPPED;
//   removeBreak(false);
}

void step3ph_motor::removeBreak(bool status){
//   if(status){
//      HAL_GPIO_WritePin(Brake_GPIO_Port, Brake_Pin, GPIO_PIN_SET);
//   }else{
//      HAL_GPIO_WritePin(Brake_GPIO_Port, Brake_Pin, GPIO_PIN_RESET);
//   }
}

void step3ph_motor::goTo(int steps, dir direct){
//  Direction = direct;
//  if(Direction == dir::CW){
//    HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
//  }else if(Direction == dir::CCW){
//    HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
//  }
  
   // установить ток для движения
  
  // запустить таймер частоты
}

void step3ph_motor::Init(){
   // init variables
   MotonStatus = motion::stopped;
  
   uint32_t Data = 64; //(uint32_t)map(CurrenrMax, 0, 3000, 0, 4095);
   HAL_DAC_Start(Dac, Channel);
   HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, Data);
   
//   if(Direction == dir::CW){
//     HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
//   }else if(Direction == dir::CCW){
//     HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
//   }
   
   if(StepMode == step::HALF){
     HAL_GPIO_WritePin(H_F_GPIO_Port, H_F_Pin, GPIO_PIN_SET);
   }else if(StepMode == step::FULL){
     HAL_GPIO_WritePin(H_F_GPIO_Port, H_F_Pin, GPIO_PIN_RESET);
   }  
   
//   HAL_GPIO_WritePin(CTRL_GPIO_Port, CTRL_Pin, GPIO_PIN_SET); // PWM in ABCD
//   
//   HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
//   
//   HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET); // enable chip

   HAL_TIM_Base_Start_IT(TimCountSteps);   
} 


//handlers*******************************************************
void step3ph_motor::StepsHandler(int steps){
  // остановить таймер частоты
  
  //установить ток удержания
}

void step3ph_motor::SensHandler(){
  
}

// обработчик таймера разгона торможения
void step3ph_motor::AccelHandler(){
   
}

//*******************************************************
step3ph_motor::step3ph_motor(){
   
}

step3ph_motor::step3ph_motor(dir direction, step stepmode, unsigned int accel, DAC_HandleTypeDef *dac, uint32_t channel,
                      TIM_HandleTypeDef *timCount, TIM_HandleTypeDef *timFreq, TIM_HandleTypeDef *timAccel) :
 base_motor(direction, stepmode, accel), Dac(dac), Channel(channel), TimCountSteps(timCount), TimFrequencies(timFreq), TimAcceleration(timAccel){
   
}
