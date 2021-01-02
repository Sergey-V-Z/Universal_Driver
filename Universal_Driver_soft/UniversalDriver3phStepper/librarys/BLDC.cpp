#include "motor.hpp"

/***************************************************************************
* Класс для Brushless DC motor (BLD) трехфазного
*
* В этом классе реализован цикл управления и контроля безколекторного двигателя
****************************************************************************/
//methods for set***********************************************
void BLDC_motor::setSpeed(uint8_t percent){
   if(percent >100){percent = 100;}
   pwm = (uint16_t) map(percent, 0, 100, 215, 361);
}

void BLDC_motor::setCurrent(uint32_t mAmax){
  CurrenrMax = mAmax;
}

//methods for get***********************************************
uint32_t BLDC_motor::get_pos(){
  return 0;
}

dir BLDC_motor::getStatusDirect(){
   return Direction;
}

bool BLDC_motor::getStatusRotation(){
   return StartMotor;
}

uint16_t BLDC_motor::getRPM(){
   return (uint16_t) RPM;
}

//methods for aktion********************************************
void BLDC_motor::start(){
   
//   removeBreak(true);
   StartMotor = true;
   SensHandler();
   
   // Acceleration will be here
   
}

void BLDC_motor::stop(){
   
   ENA_PWM = 0;
   ENB_PWM = 0;
   ENC_PWM = 0;
   StartMotor = false;
   RPM = 0;
//   removeBreak(false);
}

void BLDC_motor::removeBreak(bool status){
   if(status){
      HAL_GPIO_WritePin(Brake_GPIO_Port, Brake_Pin, GPIO_PIN_SET);
   }else{
      HAL_GPIO_WritePin(Brake_GPIO_Port, Brake_Pin, GPIO_PIN_RESET);
   }
}

void BLDC_motor::goTo(int steps, dir direct){
  
}

void BLDC_motor::Init(){
   HAL_TIM_PWM_Start(TIM, TIM_CHANNEL_2); // start PWM A
   HAL_TIM_PWM_Start(TIM, TIM_CHANNEL_3); // start PWM B
   HAL_TIM_PWM_Start(TIM, TIM_CHANNEL_1); // start PWM C 
}

//handlers******************************************************
void BLDC_motor::SensHandler(){
   position = 0;
   
   if(sens1_GPIO_Port->IDR & sens1_Pin){
      position |= 1<<0;
   }else{
      position |= 0<<0;
   }
   if(sens2_GPIO_Port->IDR & sens2_Pin){
      position |= 1<<1;
   }else{
      position |= 0<<1;
   }
   if(sens3_GPIO_Port->IDR & sens3_Pin){ 
      position |= 1<<2;
   }else{
      position |= 0<<2;
   }
   
   if(StartMotor){ 
      switch(Direction)
      {
        case dir::CW:
         {
            switch(position)
            {
              case 0b101:
               {
                  ENA_PWM = pwm;
                  ENB_PWM = pwm;
                  ENC_PWM = 0;
                  //change state
                  HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(IN2_B_GPIO_Port, IN2_B_Pin, GPIO_PIN_RESET);
                  //HAL_GPIO_WritePin(IN3_C_GPIO_Port, IN3_C_Pin, GPIO_PIN_RESET);  
                  break;
               }
              case 0b001:
               {
                  ENA_PWM = pwm;
                  ENB_PWM = 0;
                  ENC_PWM = pwm;
                  //change state
                  HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_SET);
                  //HAL_GPIO_WritePin(IN2_B_GPIO_Port, IN2_B_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(IN3_C_GPIO_Port, IN3_C_Pin, GPIO_PIN_RESET);                  
                  break;
               }
              case 0b011:
               {
                  ENA_PWM = 0;
                  ENB_PWM = pwm;
                  ENC_PWM = pwm;
                  //change state
                  //HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(IN2_B_GPIO_Port, IN2_B_Pin, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(IN3_C_GPIO_Port, IN3_C_Pin, GPIO_PIN_RESET);                  
                  break;
               }
              case 0b010:
               {
                  ENA_PWM = pwm;
                  ENB_PWM = pwm;
                  ENC_PWM = 0;
                  //change state
                  HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(IN2_B_GPIO_Port, IN2_B_Pin, GPIO_PIN_SET);
                  //HAL_GPIO_WritePin(IN3_C_GPIO_Port, IN3_C_Pin, GPIO_PIN_RESET);                  
                  break;
               }            
              case 0b110:
               {
                  ENA_PWM = pwm;
                  ENB_PWM = 0;
                  ENC_PWM = pwm;
                  //change state
                  HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
                  //HAL_GPIO_WritePin(IN2_B_GPIO_Port, IN2_B_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(IN3_C_GPIO_Port, IN3_C_Pin, GPIO_PIN_SET);                  
                  break;
               }           
              case 0b100:
               {
                  ENA_PWM = 0;
                  ENB_PWM = pwm;
                  ENC_PWM = pwm;
                  //change state
                  //HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(IN2_B_GPIO_Port, IN2_B_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(IN3_C_GPIO_Port, IN3_C_Pin, GPIO_PIN_SET);                  
                  break;
               }
            }   
            break;
         }
        case dir::CCW:
         {
            switch(position)
            {
              case 0b101:
               {
                  ENA_PWM = pwm;
                  ENB_PWM = pwm;
                  ENC_PWM = 0;
                  //change state
                  HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(IN2_B_GPIO_Port, IN2_B_Pin, GPIO_PIN_SET);
                  //HAL_GPIO_WritePin(IN3_C_GPIO_Port, IN3_C_Pin, GPIO_PIN_RESET);  
                  break;
               }
              case 0b001:
               {
                  ENA_PWM = pwm;
                  ENB_PWM = 0;
                  ENC_PWM = pwm;
                  //change state
                  HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
                  //HAL_GPIO_WritePin(IN2_B_GPIO_Port, IN2_B_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(IN3_C_GPIO_Port, IN3_C_Pin, GPIO_PIN_SET);                  
                  break;
               }
              case 0b011:
               {
                  ENA_PWM = 0;
                  ENB_PWM = pwm;
                  ENC_PWM = pwm;
                  //change state
                  //HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(IN2_B_GPIO_Port, IN2_B_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(IN3_C_GPIO_Port, IN3_C_Pin, GPIO_PIN_SET);                  
                  break;
               }
              case 0b010:
               {
                  ENA_PWM = pwm;
                  ENB_PWM = pwm;
                  ENC_PWM = 0;
                  //change state
                  HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(IN2_B_GPIO_Port, IN2_B_Pin, GPIO_PIN_RESET);
                  //HAL_GPIO_WritePin(IN3_C_GPIO_Port, IN3_C_Pin, GPIO_PIN_RESET);                  
                  break;
               }            
              case 0b110:
               {
                  ENA_PWM = pwm;
                  ENB_PWM = 0;
                  ENC_PWM = pwm;
                  //change state
                  HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_SET);
                  //HAL_GPIO_WritePin(IN2_B_GPIO_Port, IN2_B_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(IN3_C_GPIO_Port, IN3_C_Pin, GPIO_PIN_RESET);                  
                  break;
               }           
              case 0b100:
               {
                  ENA_PWM = 0;
                  ENB_PWM = pwm;
                  ENC_PWM = pwm;
                  //change state
                  //HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(IN2_B_GPIO_Port, IN2_B_Pin, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(IN3_C_GPIO_Port, IN3_C_Pin, GPIO_PIN_RESET);                  
                  break;
               }
            } 
            break;
         }   
      }
      
      //calc RPM
      if(oldTick == 0){
         oldTick = HAL_GetTick();
      }else{
         uint32_t newTick = HAL_GetTick();
         RPM = (1000.0/((newTick - oldTick)*12.0))*60;
         oldTick = newTick;
      }
   }   
}


BLDC_motor::BLDC_motor(TIM_HandleTypeDef *tim, dir direction, step stepmode, unsigned int accel) : base_motor(direction, stepmode, accel), TIM(tim){
   
}

BLDC_motor::BLDC_motor(TIM_HandleTypeDef *tim) : TIM(tim){
   
}
