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
#define PWM_Frequency 20000u // частота ШИМ
//#define PWM_MODE2

extern uint32_t count_tic;

//methods for set***********************************************
void soft_stepper::SetSpeed(uint8_t percent){

}

void soft_stepper::SetCurrent(uint32_t percent){
   if(percent >100){percent = 100;}
   PWM = (uint16_t) map(percent, 1, 100, minPWM, maxPWM);
}

void soft_stepper::SetPWM_Mode(uint32_t mod){
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

void soft_stepper::SetCurrentStop(unsigned int current){
  CurrenrSTOP = current;
}
void soft_stepper::SetCurrentMax(unsigned int current){
   
}
void soft_stepper::SetPWRstatus(bool low){
  lowpwr = low;
}

//methods for get***********************************************
uint32_t soft_stepper::get_pos(){
   return 0;
}

dir soft_stepper::getStatusDirect(){
   return Direction;
}

statusMotor soft_stepper::getStatusRotation(){
   return Status;
}

uint16_t soft_stepper::getRPM(){
   return 0;
}

//methods for aktion********************************************
void soft_stepper::start(){
   
   //   removeBreak(true);
   Status = statusMotor::ACCEL;
   FreqSteps->Instance->ARR = ConstMinAccel;
   counterSteps = 0;
   iterationsForFullAccel = (ConstMinAccel - ConstMaxAccel) / Acceleration;
   iterationsForFullPWM = (maxPWM - minPWM) / iterationsForFullAccel;
   scaleDivisionPWM = (maxPWM - minPWM) / iterationsForFullPWM;
   SetCurrent(10);
   HAL_TIM_Base_Start_IT(FreqSteps);
   
}

void soft_stepper::stop(){
   
  ENA_PWM = 0;
  ENB_PWM = 0;
  ENC_PWM = 0;
  END_PWM = 0;
  HAL_TIM_Base_Stop_IT(FreqSteps);
  Status = statusMotor::STOPPED;
  
  
   //   removeBreak(false);
}
void soft_stepper::deceleration(){
  Status = statusMotor::BRAKING;
   //stop();
}

void soft_stepper::removeBreak(bool status){
   if(status){
      HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_SET);
   }else{
      HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_RESET);
   }
}

void soft_stepper::goTo(int steps, dir direct){
   
}

void soft_stepper::Init(){
 
   
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
   
   maxPWM = TIM_2->Instance->ARR + 1;
   minPWM = (maxPWM/100)*10;
   SetCurrent(1);
   //timOneTick = (1.0/(HAL_RCC_GetHCLKFreq()/(XorTim->Instance->PSC+1)))*2;
   //timOneTick = 1.0/(HAL_RCC_GetHCLKFreq()/(XorTim->Instance->PSC+1));
   
   HAL_TIM_Encoder_Start_IT(EncTim, TIM_CHANNEL_1);
   HAL_TIM_Encoder_Start_IT(EncTim, TIM_CHANNEL_2);
   
   //HAL_TIM_Base_Start_IT(FreqSteps);
   
   // настройка комутации
   HAL_GPIO_WritePin(SW_encDiff_GPIO_Port, SW_encDiff_Pin, GPIO_PIN_RESET);     // reset/set = diff/nodiff
   HAL_GPIO_WritePin(SW_s_h_GPIO_Port, SW_s_h_Pin, GPIO_PIN_RESET);             // reset/set = soft/hard(stepper)
   
   // включение преобразователей
   HAL_GPIO_WritePin(EN_hard_GPIO_Port, EN_hard_Pin, GPIO_PIN_RESET);     // reset/set = off/on
   HAL_GPIO_WritePin(EN_soft_GPIO_Port, EN_soft_Pin, GPIO_PIN_SET);     // reset/set = off/on
   
}

//handlers******************************************************
void soft_stepper::SensHandler(){
 
}
void soft_stepper::HalfStep(){
  DWT->CYCCNT = 0;// обнуляем значение
  
  switch(Position)
    {
     case 1:
      {
        // A +
        // B -(pwm)
        // C z
        // D z
        
        A_PWM = maxPWM;
        B_PWM = 0;     
        C_PWM = 0;
        D_PWM = 0;       
        
        ENA_PWM = maxPWM;
        ENB_PWM = PWM;
        ENC_PWM = 0;
        END_PWM = 0;
        Position++;
        break;
      }
     case 2:
      {
        // A +
        // B -(pwm)
        // C +
        // D -(pwm)
        A_PWM = maxPWM;
        B_PWM = 0;     
        C_PWM = maxPWM;
        D_PWM = 0;       
        
        ENA_PWM = maxPWM;
        ENB_PWM = PWM;
        ENC_PWM = maxPWM;
        END_PWM = PWM;
        Position++;
        break;
      }
     case 3:
      {
        // A z
        // B z
        // C +
        // D -(pwm)
        A_PWM = maxPWM;
        B_PWM = 0;     
        C_PWM = maxPWM;
        D_PWM = 0;       
        
        ENA_PWM = 0;
        ENB_PWM = 0;
        ENC_PWM = maxPWM;
        END_PWM = PWM;         
        Position++;
        break;
      }
     case 4:
      {
        // A -(pwm)
        // B +
        // C +
        // D -(pwm)
        A_PWM = 0;
        B_PWM = maxPWM;     
        C_PWM = maxPWM;
        D_PWM = 0;       
        
        ENA_PWM = PWM;
        ENB_PWM = maxPWM;
        ENC_PWM = maxPWM;
        END_PWM = PWM;          
        Position++;
        break;
      }
     case 5:
      {
        // A -(pwm)
        // B +
        // C z
        // D z
        A_PWM = 0;
        B_PWM = maxPWM;     
        C_PWM = maxPWM;
        D_PWM = 0;       
        
        ENA_PWM = PWM;
        ENB_PWM = maxPWM;
        ENC_PWM = 0;
        END_PWM = 0;         
        Position++;
        break;
      }
     case 6:
      {
        // A -(pwm)
        // B +
        // C -(pwm)
        // D + 
        // A -(pwm)
        // B +
        // C z
        // D z
        A_PWM = 0;
        B_PWM = maxPWM;     
        C_PWM = 0;
        D_PWM = maxPWM;       
        
        ENA_PWM = PWM;
        ENB_PWM = maxPWM;
        ENC_PWM = PWM;
        END_PWM = maxPWM;
        Position++;
        break;
      }
     case 7:
      {
        // A z
        // B z
        // C -(pwm)
        // D +  
        A_PWM = 0;
        B_PWM = maxPWM;     
        C_PWM = 0;
        D_PWM = maxPWM;       
        
        ENA_PWM = 0;
        ENB_PWM = 0;
        ENC_PWM = PWM;
        END_PWM = maxPWM;         
        Position++;
        break;
      }
     case 8:
      {
        // A +
        // B -(pwm)
        // C -(pwm)
        // D + 
        A_PWM = maxPWM;
        B_PWM = 0;     
        C_PWM = 0;
        D_PWM = maxPWM;       
        
        ENA_PWM = maxPWM;
        ENB_PWM = PWM;
        ENC_PWM = PWM;
        END_PWM = maxPWM;         
        Position = 1;
        break;
      }
     default:
      {
        Position = 1;
        break;
      }
      
    }
  
  
  count_tic = DWT->CYCCNT;//смотрим сколько натикал  
}
void soft_stepper::FullStep(){

}
void soft_stepper::StepsHandler(int steps){
       
}

void soft_stepper::AccelHandler(){
   
}
//счетчик обшего количества шагов
void soft_stepper::StepsAllHandler(int steps){
  counterSteps++;
  switch(Status)
  {
    case statusMotor::ACCEL:
    {

      //если нельзя больше ускорить то ствам максимут иначе ускоряем
      if(FreqSteps->Instance->ARR - Acceleration <= MaxAccel)
        {
          FreqSteps->Instance->ARR = MaxAccel;
          Status = statusMotor::MOTION;
        }
      else
        {
          FreqSteps->Instance->ARR -= Acceleration;
          
          // добавить увеличение тока по заранее расчитанным данным
        }
      
      break;
    }
    case statusMotor::MOTION:
    {
      break;
    }
   case statusMotor::BRAKING:
    {
      break;
    }
   case statusMotor::STOPPED:
    {
      break;
    }
    default:
    {
      break;
    }
    
  }

  
}
// шим по верхнему ключу
uint32_t soft_stepper::PWM_Mode_0(){
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
               ENB_PWM = maxPWM;
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
           case 0b100: // B-C+
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
uint32_t soft_stepper::PWM_Mode_1(){
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
               
               ENB_PWM = PWM;
               ENA_PWM = maxPWM;
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
               
               ENC_PWM = PWM;
               ENA_PWM = maxPWM;
               
               break;
            }
           case 0b011://B+C-
            {
               ENA_PWM = 0;
               
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
uint32_t soft_stepper::PWM_Mode_2(){
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

soft_stepper::soft_stepper(TIM_HandleTypeDef *tim_1, TIM_HandleTypeDef *tim_2, TIM_HandleTypeDef *freqSteps, TIM_HandleTypeDef *encTim, dir direction, step stepmode, unsigned int accel) : base_motor(direction, stepmode, accel), TIM_1(tim_1){
   
}

soft_stepper::soft_stepper(TIM_HandleTypeDef *tim_1, TIM_HandleTypeDef *tim_2, TIM_HandleTypeDef *freqSteps, TIM_HandleTypeDef *encTim) : TIM_1(tim_1), TIM_2(tim_2), FreqSteps(freqSteps), EncTim(encTim){
   
}
