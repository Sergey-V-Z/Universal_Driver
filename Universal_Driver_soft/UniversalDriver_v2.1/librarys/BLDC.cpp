#include "motor.hpp"

/***************************************************************************
* Класс для Brushless DC motor (BLD) трехфазного
*
* В этом классе реализован цикл управления и контроля безколекторного двигателя
****************************************************************************/
// Определения каналов
#define A_PWM TIM_1->Instance->CCR4
#define ENA_PWM TIM_1->Instance->CCR3
#define B_PWM TIM_1->Instance->CCR2
#define ENB_PWM TIM_1->Instance->CCR1

#define C_PWM TIM_2->Instance->CCR4
#define ENC_PWM TIM_2->Instance->CCR3
#define D_PWM TIM_2->Instance->CCR2
#define END_PWM TIM_2->Instance->CCR1

// другие настройки
#define PWM_Frequency 15000u // частота ШИМ
//#define PWM_MODE2

extern uint32_t count_tic;

//methods for set***********************************************
void BLDC_motor::setSpeed(uint16_t percent){
   if(percent >1000){percent = 1000;}
   currentSpeed = percent;
   PWM = (uint16_t) map(percent, 1, 1000, minPWM, maxPWM);
}

void BLDC_motor::changeSpeed(uint16_t percent){
   if(percent >1000){percent = 1000;}
   finalSpeed = percent;
   currentSpeed = (uint16_t) map(PWM, minPWM, maxPWM, 1, 1000);
   
   if(finalSpeed > currentSpeed){
      upSpeed = true;
      startTimer = true;
   }else if(finalSpeed < currentSpeed){
      upSpeed = false;
      startTimer = true;
   }else if(finalSpeed == currentSpeed){
      startTimer = false;
      PWM = (uint16_t) map(finalSpeed, 1, 1000, minPWM, maxPWM);
   }
   //PWM = (uint16_t) map(percent, 1, 1000, minPWM, maxPWM);
}

void BLDC_motor::setCurrent(uint32_t mAmax){
   CurrenrMax = mAmax;
}

void BLDC_motor::setPWM_Mode(uint32_t mod){
   switch(mod)
   {
     case 0: // по нижнему ключу
      {
         PWM_Mode = mod;
         break;
      }
     case 1: // по верхнему ключу 
      {
         PWM_Mode = mod;
         break;
      }
     case 2: // по обоим ключам
      {
         PWM_Mode = mod;
         break;
      }
     default: // ошибка
      {
         break;
      }
      
   }
   
}

//methods for get***********************************************
uint32_t BLDC_motor::get_pos(){
   return 0;
}

dir BLDC_motor::getStatusDirect(){
   return Direction;
}

statusMotor BLDC_motor::getStatusRotation(){
   return Status;
}

uint16_t BLDC_motor::getRPM(){
   return (uint16_t) RPM;
}

//methods for aktion********************************************
void BLDC_motor::start(){
   
   //   removeBreak(true);
   Status = statusMotor::MOTION;
   //ForcedRotation();
   //setSpeed(1000);
   uint16_t temp_PWM = PWM;
   PWM = maxPWM; //(uint16_t) map(percent, 1, 1000, minPWM, maxPWM);
   if(SensHandler()){
      ForcedRotation();
      if(SensHandler()){
         stop();
      }
   }
   
   PWM = temp_PWM;
   
   // Acceleration will be here
   
}

void BLDC_motor::stop(){
   
   ENA_PWM = 0;
   ENB_PWM = 0;
   ENC_PWM = 0;
   Status = statusMotor::STOPPED;
   RPM = 0;
   //   removeBreak(false);
}
void BLDC_motor::deceleration(){
   stop();
}

void BLDC_motor::removeBreak(bool status){
   if(status){
      HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_SET);
   }else{
      HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_RESET);
   }
}

void BLDC_motor::goTo(int steps, dir direct){
   
}

void BLDC_motor::Init(){
   
   
   //режим подачи шима 
   PWM_Mode = 1; // на нижние ключи
   
   //установка частоты ШИМ
   TIM_1->Instance->PSC = (HAL_RCC_GetHCLKFreq()/(PWM_Frequency*360))-1;
   TIM_2->Instance->PSC = (HAL_RCC_GetHCLKFreq()/(PWM_Frequency*360))-1;
   
   HAL_TIM_PWM_Start(TIM_1, TIM_CHANNEL_1); 
   HAL_TIM_PWM_Start(TIM_1, TIM_CHANNEL_2); 
   HAL_TIM_PWM_Start(TIM_1, TIM_CHANNEL_3);  
   HAL_TIM_PWM_Start(TIM_1, TIM_CHANNEL_4); 
   
   HAL_TIM_PWM_Start(TIM_2, TIM_CHANNEL_1); 
   HAL_TIM_PWM_Start(TIM_2, TIM_CHANNEL_2); 
   HAL_TIM_PWM_Start(TIM_2, TIM_CHANNEL_3); 
   HAL_TIM_PWM_Start(TIM_2, TIM_CHANNEL_4); 
   
   maxPWM = (TIM_2->Instance->ARR + 1); 
   minPWM = ((TIM_2->Instance->ARR + 1)/100) * 10; // 10%
   setSpeed(200);
   
   timOneTick = (1.0/(HAL_RCC_GetHCLKFreq()/(XorTim->Instance->PSC+1)))*2;
   //timOneTick = 1.0/(HAL_RCC_GetHCLKFreq()/(XorTim->Instance->PSC+1));
   
   HAL_TIM_Encoder_Start_IT(EncTim, TIM_CHANNEL_1);
   HAL_TIM_Encoder_Start_IT(EncTim, TIM_CHANNEL_2);
   
   HAL_TIM_IC_Start_IT(XorTim, TIM_CHANNEL_1);
   
   // настройка комутации
   HAL_GPIO_WritePin(SW_encDiff_GPIO_Port, SW_encDiff_Pin, GPIO_PIN_RESET);     // reset/set = diff/nodiff
   HAL_GPIO_WritePin(SW_s_h_GPIO_Port, SW_s_h_Pin, GPIO_PIN_RESET);             // reset/set = soft/hard(stepper)
   
   // включение преобразователей
   HAL_GPIO_WritePin(EN_hard_GPIO_Port, EN_hard_Pin, GPIO_PIN_RESET);     // reset/set = off/on
   HAL_GPIO_WritePin(EN_soft_GPIO_Port, EN_soft_Pin, GPIO_PIN_SET);     // reset/set = off/on
   
}

//handlers******************************************************
bool BLDC_motor::SensHandler(){
   DWT->CYCCNT = 0;// обнуляем значение
   
   Position = 0;
   uint32_t ret = 0;
   if(hal1_GPIO_Port->IDR & hal1_Pin){
      Position |= 1<<0;
   }else{
      Position &= ~(1<<0);
   }
   if(hal2_GPIO_Port->IDR & hal2_Pin){
      Position |= 1<<1;
   }else{
      Position &= ~(1<<1);
   }
   if(hal3_GPIO_Port->IDR & hal3_Pin){ 
      Position |= 1<<2;
   }else{
      Position &= ~(1<<2);
   }
   
   if(Status == statusMotor::MOTION){ 
      
      switch(PWM_Mode)
      {
        case 0:
         {
            ret = PWM_Mode_0();
            break;
         }
        case 1:
         {
            ret = PWM_Mode_1();
            break;
         }
        case 2:
         {
            ret = PWM_Mode_2();
            break;
         }
        default:
         {
            break;
         }
      }
      // останов если функция вернула ошибку
//      if(ret){
//         // прогнать мотор по кругу без датчиков
//         stop();
//      }
      //сброс таймаута
      timeout = 0;
      //calc RPM
      //      if(oldTick == 0){
      //         oldTick = HAL_GetTick();
      //      }else{
      //         uint32_t newTick = HAL_GetTick();
      //         RPM = (1000.0/((newTick - oldTick)*12.0))*60;
      //         oldTick = newTick;
      //      }
      if(counterSteps == 6){
         RPM = (int)(1.0/(currentTimeTurn * timOneTick))*60;
         counterSteps = 0;
         currentTimeTurn = 0;
         
      }else{
         counterSteps++;
         currentTimeTurn += XorTim->Instance->CCR1;
      }
      //RPM = (int)(1.0/(XorTim->Instance->CCR1 * timOneTick))*60;
   }
   
   count_tic = DWT->CYCCNT;//смотрим сколько натикал   
   return ret;
}

// Обработчик ускорения
void BLDC_motor::AccelHandler(){
   //если таймер запущен
   if(startTimer == true){
      if(upSpeed){
         if(currentSpeed + accelerationPercent >= finalSpeed){
            currentSpeed = finalSpeed;
            PWM = (uint16_t) map(finalSpeed, 1, 1000, minPWM, maxPWM); 
            startTimer = false;
            
         }else if (currentSpeed + accelerationPercent < finalSpeed){
            currentSpeed += accelerationPercent;
            PWM = (uint16_t) map(currentSpeed, 1, 1000, minPWM, maxPWM);
            
         }  
      }else{
         if(currentSpeed - accelerationPercent <= finalSpeed){
            currentSpeed = finalSpeed;
            PWM = (uint16_t) map(finalSpeed, 1, 1000, minPWM, maxPWM); 
            startTimer = false;
            
         }else if (currentSpeed - accelerationPercent > finalSpeed){
            currentSpeed -= accelerationPercent;
            PWM = (uint16_t) map(currentSpeed, 1, 1000, minPWM, maxPWM);
            
         }      
      }

   }
}

void BLDC_motor::ForcedRotation(){
   
   forcedRotation = true;
   Position = 1;
   for(int i = 0; i < 6; ++i)
   {
    PWM_Mode_1();
    Position++;
   }

   forcedRotation = false;
}

// шим по верхнему ключу
uint32_t BLDC_motor::PWM_Mode_0(){
   uint32_t ret = 0;
   switch(Direction)
   {
     case dir::CW:
      {
         switch(Position)
         {
           case 0b101://A+B-
            {
               ENC_PWM = 0;
               
               A_PWM = maxPWM;
               B_PWM = 0;     
               //С_PWM = 0;
               
               //delay_us(delayComm);
               
               ENA_PWM = PWM;
               ENB_PWM = maxPWM;
               //ENC_PWM = 0;
               
               break;
            }
           case 0b001://A+C-
            {
               
               ENB_PWM = 0;
               
               A_PWM = maxPWM;
               //B_PWM = 0;
               C_PWM = 0;  
               
               //delay_us(delayComm);
               
               ENA_PWM = PWM;
               ENC_PWM = maxPWM;
               
               break;
            }
           case 0b011://B+C-
            {
               ENA_PWM = 0;
               
               //A_PWM = 0;
               B_PWM = maxPWM;
               C_PWM = 0;
               
               //delay_us(delayComm);
               
               ENB_PWM = PWM;
               ENC_PWM = maxPWM;
               
               break; 
            }
           case 0b010://A-B+
            {
               
               ENC_PWM = 0;
               
               A_PWM = 0;
               B_PWM = maxPWM;
               //С_PWM = 0;                   
               
               //delay_us(delayComm);
               
               ENA_PWM = maxPWM;
               ENB_PWM = PWM;
               //ENC_PWM = 0;
               
               break;
            }            
           case 0b110://A-C+
            {
               ENB_PWM = 0;
               
               A_PWM = 0;
               //B_PWM = 0;
               C_PWM = maxPWM;                   
               
               //delay_us(delayComm);
               
               ENA_PWM = maxPWM;
               ENC_PWM = PWM;
               
               break;
            }           
           case 0b100:// B-C+
            {
               ENA_PWM = 0;
               
               //A_PWM = 0;
               B_PWM = 0;
               C_PWM = maxPWM;        
               
               //delay_us(delayComm);
               
               //ENA_PWM = 0;
               ENB_PWM = maxPWM;
               ENC_PWM = PWM;
               
               break;
            }
           default:
            {
               ret = 1;
               break;
            }
         }   
         break;
      }
     case dir::CCW:
      {
         switch(Position)
         {
           case 0b101: // A-B+
            {
               ENC_PWM = 0;
               
               A_PWM = 0;
               B_PWM = maxPWM;
               //С_PWM = 0;                  
               
               //delay_us(delayComm);
               
               ENA_PWM = maxPWM;
               ENB_PWM = PWM;
               //ENC_PWM = 0;
               
               break;
            }
           case 0b001: // A-C+
            {
               ENB_PWM = 0;
               
               A_PWM = 0;
               //B_PWM = 0;
               C_PWM = maxPWM;                 
               
               //delay_us(delayComm);
               
               ENA_PWM = maxPWM;
               //ENB_PWM = 0;
               ENC_PWM = PWM;               
               
               break;
            }
           case 0b011: // B-C+
            {
               ENA_PWM = 0;
               
               //A_PWM = 0;
               B_PWM = 0;
               C_PWM = maxPWM;                  
               
               //delay_us(delayComm);
               
               //ENA_PWM = 0;
               ENB_PWM = maxPWM;
               ENC_PWM = PWM;
               
               break;
            }
           case 0b010: // A+B-
            {
               ENC_PWM = 0;
               
               A_PWM = maxPWM;
               B_PWM = 0;
               //С_PWM = 0;                   
               
               //delay_us(delayComm);
               
               ENA_PWM = PWM;
               ENB_PWM = maxPWM;
               //ENC_PWM = 0;
               
               break;
            }            
           case 0b110: // A+C-
            {
               ENB_PWM = 0;
               
               A_PWM = maxPWM;
               //B_PWM = 0;
               C_PWM = 0;    
               
               //delay_us(delayComm);
               
               ENA_PWM = PWM;
               //ENB_PWM = 0;
               ENC_PWM = maxPWM;
               
               break;
            }           
           case 0b100: // B+C-
            {
               ENA_PWM = 0;
               
               //A_PWM = 0;
               B_PWM = maxPWM;
               C_PWM = 0;   
               
               //delay_us(delayComm);
               
               //ENA_PWM = 0;
               ENB_PWM = PWM;
               ENC_PWM = maxPWM;
               
               break;
            }
           default:
            {
               ret = 1;
               break;
            }
         } 
         break;
      }   
   }
   return ret;
}

// шим по нижнему ключу
uint32_t BLDC_motor::PWM_Mode_1(){
   uint32_t ret = 0;
   switch(Direction)
   {
     case dir::CW:
      {
         switch(Position)
         {
           case 0b101: //A+B-
            {
               ENC_PWM = 0;
               // задержка для выключения 
               BLDC_Delay(22);
               
               A_PWM = maxPWM;
               B_PWM = 0;     
               //С_PWM = 0;
               
               //delay_us(delayComm);
               
               ENB_PWM = PWM;
               ENA_PWM = maxPWM;
               //ENC_PWM = 0;
               
               break;
            }
           case 0b001: //A+C-
            {
               
               ENB_PWM = 0;
               // задержка для выключения 
               BLDC_Delay(22);
               
               A_PWM = maxPWM;
               //B_PWM = 0;
               C_PWM = 0;  
               
               //delay_us(delayComm);
               
               ENC_PWM = PWM;
               ENA_PWM = maxPWM;
               
               break;
            }
           case 0b011://B+C-
            {
               ENA_PWM = 0;
               // задержка для выключения 
               BLDC_Delay(22);
               
               //A_PWM = 0;
               B_PWM = maxPWM;
               C_PWM = 0;
               
               //delay_us(delayComm);
               
               ENC_PWM = PWM;
               ENB_PWM = maxPWM;
               
               break; 
            }
           case 0b010://A-B+
            {
               
               ENC_PWM = 0;
               // задержка для выключения 
               BLDC_Delay(22);
               
               A_PWM = 0;
               B_PWM = maxPWM;
               //С_PWM = 0;                   
               
               //delay_us(delayComm);
               
               ENB_PWM = maxPWM;
               ENA_PWM = PWM;
               //ENC_PWM = 0;
               
               break;
            }            
           case 0b110://A-C+
            {
               ENB_PWM = 0;
               // задержка для выключения 
               BLDC_Delay(22);
               
               A_PWM = 0;
               //B_PWM = 0;
               C_PWM = maxPWM;                   
               
               //delay_us(delayComm);
               
               ENC_PWM = maxPWM;
               ENA_PWM = PWM;
               
               break;
            }           
           case 0b100: // B-C+
            {
               ENA_PWM = 0;
               // задержка для выключения 
               BLDC_Delay(22);
               
               //A_PWM = 0;
               B_PWM = 0;
               C_PWM = maxPWM;        
               
               //delay_us(delayComm);
               
               //ENA_PWM = 0;
               ENC_PWM = maxPWM;
               ENB_PWM = PWM;
               
               break;
            }
           default:
            {
               ret = 1;
               break;
            }
         }   
         break;
      }
     case dir::CCW:
      {
         switch(Position)
         {
           case 0b101: // A-B+
            {
               ENC_PWM = 0;
               // задержка для выключения 
               BLDC_Delay(22);
               
               A_PWM = 0;
               B_PWM = maxPWM;
               //С_PWM = 0;                  
               
               //delay_us(delayComm);
               
               ENB_PWM = maxPWM;
               ENA_PWM = PWM;
               //ENC_PWM = 0;
               
               break;
            }
           case 0b001: // A-C+
            {
               ENB_PWM = 0;
               // задержка для выключения 
               BLDC_Delay(22);
               
               A_PWM = 0;
               //B_PWM = 0;
               C_PWM = maxPWM;                 
               
               //delay_us(delayComm);
               
               ENC_PWM = maxPWM;
               //ENB_PWM = 0;
               ENA_PWM = PWM;               
               
               break;
            }
           case 0b011: // B-C+
            {
               ENA_PWM = 0;
               // задержка для выключения 
               BLDC_Delay(22);
               
               //A_PWM = 0;
               B_PWM = 0;
               C_PWM = maxPWM;                  
               
               //delay_us(delayComm);
               
               //ENA_PWM = 0;
               ENC_PWM = maxPWM;
               ENB_PWM = PWM;
               
               break;
            }
           case 0b010: // A+B-
            {
               ENC_PWM = 0;
               // задержка для выключения 
               BLDC_Delay(22);
               
               A_PWM = maxPWM;
               B_PWM = 0;
               //С_PWM = 0;                   
               
               //delay_us(delayComm);
               
               ENB_PWM = PWM;
               ENA_PWM = maxPWM;
               //ENC_PWM = 0;
               
               break;
            }            
           case 0b110: // A+C-
            {
               ENB_PWM = 0;
               // задержка для выключения 
               BLDC_Delay(22);
               
               A_PWM = maxPWM;
               //B_PWM = 0;
               C_PWM = 0;    
               
               //delay_us(delayComm);
               
               ENC_PWM = PWM;
               //ENB_PWM = 0;
               ENA_PWM = maxPWM;
               
               break;
            }           
           case 0b100: // B+C-
            {
               ENA_PWM = 0;
               // задержка для выключения 
               BLDC_Delay(22);
               
               //A_PWM = 0;
               B_PWM = maxPWM;
               C_PWM = 0;   
               
               //delay_us(delayComm);
               
               //ENA_PWM = 0;
               ENC_PWM = PWM;
               ENB_PWM = maxPWM;
               
               break;
            }
           default:
            {
               ret = 1;
               break;
            }
         } 
         break;
      }   
   }
   return ret;
}

// шим по обоим ключам
uint32_t BLDC_motor::PWM_Mode_2(){
   uint32_t ret = 0;
   switch(Direction)
   {
     case dir::CW:
      {
         switch(Position)
         {
           case 0b101: //A+B-
            {
               ENC_PWM = 0;
               
               A_PWM = maxPWM;
               B_PWM = 0;     
               //С_PWM = 0;
               
               //delay_us(delayComm);
               
               ENA_PWM = PWM;
               ENB_PWM = PWM;
               //ENC_PWM = 0;
               
               break;
            }
           case 0b001: //A+C-
            {
               
               ENB_PWM = 0;
               
               A_PWM = maxPWM;
               //B_PWM = 0;
               C_PWM = 0;  
               
               //delay_us(delayComm);
               
               ENA_PWM = PWM;
               ENC_PWM = PWM;
               
               break;
            }
           case 0b011://B+C-
            {
               ENA_PWM = 0;
               
               //A_PWM = 0;
               B_PWM = maxPWM;
               C_PWM = 0;
               
               //delay_us(delayComm);
               
               ENB_PWM = PWM;
               ENC_PWM = PWM;
               
               break; 
            }
           case 0b010://A-B+
            {
               
               ENC_PWM = 0;
               
               A_PWM = 0;
               B_PWM = maxPWM;
               //С_PWM = 0;                   
               
               //delay_us(delayComm);
               
               ENA_PWM = PWM;
               ENB_PWM = PWM;
               //ENC_PWM = 0;
               
               break;
            }            
           case 0b110://A-C+
            {
               ENB_PWM = 0;
               
               A_PWM = 0;
               //B_PWM = 0;
               C_PWM = maxPWM;                   
               
               //delay_us(delayComm);
               
               ENA_PWM = PWM;
               ENC_PWM = PWM;
               
               break;
            }           
           case 0b100: // B-C+
            {
               ENA_PWM = 0;
               
               //A_PWM = 0;
               B_PWM = 0;
               C_PWM = maxPWM;        
               
               //delay_us(delayComm);
               
               //ENA_PWM = 0;
               ENB_PWM = PWM;
               ENC_PWM = PWM;
               
               break;
            }
           default:
            {
               ret = 1;
               break;
            }
         }   
         break;
      }
     case dir::CCW:
      {
         switch(Position)
         {
           case 0b101: // A-B+
            {
               ENC_PWM = 0;
               
               A_PWM = 0;
               B_PWM = maxPWM;
               //С_PWM = 0;                  
               
               //delay_us(delayComm);
               
               ENA_PWM = PWM;
               ENB_PWM = PWM;
               //ENC_PWM = 0;
               
               break;
            }
           case 0b001: // A-C+
            {
               ENB_PWM = 0;
               
               A_PWM = 0;
               //B_PWM = 0;
               C_PWM = maxPWM;                 
               
               //delay_us(delayComm);
               
               ENA_PWM = PWM;
               //ENB_PWM = 0;
               ENC_PWM = PWM;               
               
               break;
            }
           case 0b011: // B-C+
            {
               ENA_PWM = 0;
               
               //A_PWM = 0;
               B_PWM = 0;
               C_PWM = maxPWM;                  
               
               //delay_us(delayComm);
               
               //ENA_PWM = 0;
               ENB_PWM = PWM;
               ENC_PWM = PWM;
               
               break;
            }
           case 0b010: // A+B-
            {
               ENC_PWM = 0;
               
               A_PWM = maxPWM;
               B_PWM = 0;
               //С_PWM = 0;                   
               
               //delay_us(delayComm);
               
               ENA_PWM = PWM;
               ENB_PWM = PWM;
               //ENC_PWM = 0;
               
               break;
            }            
           case 0b110: // A+C-
            {
               ENB_PWM = 0;
               
               A_PWM = maxPWM;
               //B_PWM = 0;
               C_PWM = 0;    
               
               //delay_us(delayComm);
               
               ENA_PWM = PWM;
               //ENB_PWM = 0;
               ENC_PWM = PWM;
               
               break;
            }           
           case 0b100: // B+C-
            {
               ENA_PWM = 0;
               
               //A_PWM = 0;
               B_PWM = maxPWM;
               C_PWM = 0;   
               
               //delay_us(delayComm);
               
               //ENA_PWM = 0;
               ENB_PWM = PWM;
               ENC_PWM = PWM;
               
               break;
            }
           default:
            {
               ret = 1;
               break;
            }
         } 
         break;
      }   
   }
   return ret;}

void BLDC_motor :: BLDC_Delay(uint32_t tacts){
   for(int i = 0; i < tacts; ++i)
   {
     asm("nop");
   }
}

BLDC_motor::BLDC_motor(TIM_HandleTypeDef *tim_1, TIM_HandleTypeDef *tim_2, TIM_HandleTypeDef *xorTim, TIM_HandleTypeDef *encTim, dir direction, step stepmode, unsigned int accel) : base_motor(direction, stepmode, accel), TIM_1(tim_1){
   
}

BLDC_motor::BLDC_motor(TIM_HandleTypeDef *tim_1, TIM_HandleTypeDef *tim_2, TIM_HandleTypeDef *xorTim, TIM_HandleTypeDef *encTim) : TIM_1(tim_1), TIM_2(tim_2), XorTim(xorTim), EncTim(encTim){
   
}
