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
		//TimFrequencies->Instance->CCR1 = Speed;
	}
	Parameter_update();
}

void extern_driver::SetAcceleration(uint16_t percent){
	if(percent >1000){percent = 1000;}
	if(percent <1){percent = 1;}
	settings->Accel = (uint16_t) map(percent, 1, 1000, 1, Speed);
	Parameter_update();
}

void extern_driver::SetDeacceleration(uint16_t percent){
	if(percent >1000){percent = 1000;}
	if(percent <1){percent = 1;}
	settings->Deaccel = percent;
	Parameter_update();

}

void extern_driver::SetTarget (uint32_t temp){
	if(temp >65000){temp = 65000;}
	if(temp <1){temp = 1;}
	settings->Target = temp;
	TimCountAllSteps->Instance->ARR = temp;
	Parameter_update();
}

void extern_driver::SetZeroPoint (void){

}

void extern_driver::SetMode(bool mod) {
	modCounter = mod;
}
// расчитывает и сохраняет все параметры разгона и торможения
void extern_driver::Parameter_update(void){

	//FeedbackBraking_P1 = target - ((uint16_t) map(Deaccel, 1, 1000, 1, target));
	//FeedbackBraking_P0 = CircleCounts - ((uint16_t) map(Deaccel, 1, 1000, 1, CircleCounts - target)); // Начало торможения перед нулевой точкой в отсчетах это 1000

	//расчет шага ускорения/торможения
	//Accel = Speed / (target - FeedbackBraking_P1);
	//TimCountAllSteps->Instance->ARR = target;
}

//methods for get************************************************

uint32_t extern_driver::getAcceleration() {
	return (uint16_t) map(settings->Accel, 1, Speed, 1, 1000);
}

uint32_t extern_driver::getSpeed() {

	return (uint32_t) map(Speed, MinSpeed, MaxSpeed, 1, 1000);
}

uint32_t extern_driver::getTarget() {
	return settings->Target;
	//return TimCountAllSteps->Instance->ARR;
}
uint32_t extern_driver::get_pos(){
	return Position;
}

dir extern_driver::getStatusDirect(){
	return settings->Direct;
}

statusMotor extern_driver::getStatusRotation(){
	return Status;
}

uint16_t extern_driver::getRPM(){
	return 0;
}

bool extern_driver::getMode() {
	return modCounter;
}

//methods for aktion*********************************************
void extern_driver::start(){
	//   removeBreak(true);
	if(Status == statusMotor::STOPPED){
		//Установка направления
		if(settings->Direct == dir::CW){
			HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
		}

		TimAcceleration->Instance->CCR2 = 0;
		Status = statusMotor::ACCEL;
		HAL_TIM_OC_Start(TimFrequencies, ChannelClock);
	}
}

void extern_driver::stop(){
	//   removeBreak(false);
	if((Status == statusMotor::MOTION) || (Status == statusMotor::BRAKING)){
		HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
		(TimFrequencies->Instance->ARR) = MinSpeed;
		TimCountAllSteps->Instance->ARR = 0;
		__HAL_TIM_SET_COUNTER(TimCountAllSteps, 0);
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
		//      HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_SET);
	}else{
		//      HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_RESET);
	}
}

void extern_driver::goTo(int steps, dir direct){

}

void extern_driver::Init(settings_t *set){

	settings = set;
	// init variables

	//Расчет максималных параметров PWM для скорости
	MaxSpeed =  1;//((TimFrequencies->Instance->ARR/100)*1);
	MinSpeed =  20000;//((TimFrequencies->Instance->ARR/100)*100);
	//установка делителя
	TimFrequencies->Instance->PSC = 399; //(80 мГц/400)
	TimFrequencies->Instance->ARR = MinSpeed;
	TimCountAllSteps->Instance->ARR = settings->Target;
	//Accel = 1000;
	//Speed = MinSpeed;
	SetSpeed(100); //10%

	//установка скорости калибровки
	Speed_Call = (uint16_t) map(15, 1, 1000, MinSpeed, MaxSpeed);

	Status = statusMotor::STOPPED;
	FeedbackType = fb::ENCODER; // сделать установку этого значения из настроек

	//SetAcceleration(settings.Accel); // ускорение
	//SetDeacceleration(settings->Deaccel);


	//HAL_DAC_Start(Dac, Channel);
	//HAL_DAC_SetValue(Dac, Channel, DAC_ALIGN_12B_R, CurrenrSTOP);

	if(settings->Direct == dir::CW){
		HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
	}else if(settings->Direct == dir::CCW){
		HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
	}

	if(StepMode == step::HALF){
		//     HAL_GPIO_WritePin(H_F_GPIO_Port, H_F_Pin, GPIO_PIN_SET);
		//TimCountAllSteps->Instance->PSC = 1;
	}else if(StepMode == step::FULL){
		//     HAL_GPIO_WritePin(H_F_GPIO_Port, H_F_Pin, GPIO_PIN_RESET);
		//TimCountAllSteps->Instance->PSC = 3;
	}

	//TimFrequencies->Instance->PSC =
	__HAL_TIM_SET_COUNTER(TimCountAllSteps, 0);
	//HAL_TIM_Encoder_Start_IT(TimCountAllSteps, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(TimCountAllSteps);
	//HAL_TIM_PWM_Start(TimFrequencies, ChannelClock);
	//HAL_TIM_OC_Start(TimFrequencies, ChannelClock);

	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET); // enable chip

	Parameter_update();
} 


//handlers*******************************************************
//счетчик операционный для счета шагов между операциями
void extern_driver::StepsHandler(uint32_t steps){

}

//счетчик обшего количества шагов
void extern_driver::StepsAllHandler(uint32_t steps){
	if(modCounter){
		this->stop();
	}

/*
	if(steps <= 1){
		//steps = 1000;
		__HAL_TIM_SET_COUNTER(TimCountAllSteps, 1000);
	}
	if(steps >= 2001){
		//steps = 1000;
		__HAL_TIM_SET_COUNTER(TimCountAllSteps, 1000);
	}
 */
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
			if((TimFrequencies->Instance->ARR) > Speed){ // если "ускорение" меньше или ровно максимальному то выставить максимум
				//если разница меньше нуля
				if(TimFrequencies->Instance->ARR < settings->Accel){
					(TimFrequencies->Instance->ARR) = Speed;
				}else{
					(TimFrequencies->Instance->ARR) -= settings->Accel; // Ускоряем
				}

			}else{
				(TimFrequencies->Instance->ARR) = Speed;
				Status = statusMotor::MOTION;
			}
			break;
		}
		case statusMotor::BRAKING:
		{
			if((TimFrequencies->Instance->ARR) < MinSpeed){ // если "торможение" больше или ровно минимальному то выставить минимум и остоновить торможение
				//проверить на переполнение
				(TimFrequencies->Instance->ARR) += settings->Accel;
			}else{
				(TimFrequencies->Instance->ARR) = MinSpeed;
				//Status = statusMotor::STOPPED;
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

extern_driver::extern_driver(TIM_HandleTypeDef *timCount, TIM_HandleTypeDef *timFreq, uint32_t channelFreq , TIM_HandleTypeDef *timAccel) :
								TimCountAllSteps(timCount), TimFrequencies(timFreq), ChannelClock(channelFreq), TimAcceleration(timAccel)
{

}

extern_driver::~extern_driver(){

}

void extern_driver::SetDirection(dir direction) {
	settings->Direct = direction;
}

double extern_driver::map(double x, double in_min, double in_max,
		double out_min, double out_max) {
	 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
