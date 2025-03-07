#include "motor.hpp"
#include "stdio.h"

/***************************************************************************
 * Класс для шагового двухфазного мотора
 *
 * В этом классе реализован цикл управления и контроля шагового двигателя
 ****************************************************************************/

void extern_driver::Init(settings_t *set){

	// init variables
	settings = set;
	/*
	TimFrequencies->Instance->ARR = 10000;
	//Расчет максималных параметров PWM для скорости
	MaxSpeed =  (TimFrequencies->Instance->ARR*1); // 100%
	MinSpeed =  (TimFrequencies->Instance->ARR*0.007); 0.7%
	*/

	//Расчет максималных параметров PWM для скорости
	MaxSpeed = 50; //((TimFrequencies->Instance->ARR/100)*1);
	MinSpeed = 13000; //((TimFrequencies->Instance->ARR/100)*100);

	//установка делителя
	TimFrequencies->Instance->PSC = 11; //(80 мГц/400) //399
	TimFrequencies->Instance->ARR = MinSpeed;

	SetSpeed(100); //10%


	Status = statusMotor::STOPPED;
	FeedbackType = fb::ENCODER; // сделать установку этого значения из настроек

	if(settings->Direct == dir::CW){
		HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
	}else if(settings->Direct == dir::CCW){
		HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
	}
/*
	switch (ChannelClock) {
	case TIM_CHANNEL_1:
		(TimFrequencies->Instance->CCR1) = 0;
		break;
	case TIM_CHANNEL_2:
		(TimFrequencies->Instance->CCR2) = 0;
		break;
	case TIM_CHANNEL_3:
		(TimFrequencies->Instance->CCR3) = 0;
		break;
	case TIM_CHANNEL_4:
		(TimFrequencies->Instance->CCR3) = 0;
		break;
	default:
		break;
	}*/

	//HAL_TIM_PWM_Start(TimFrequencies, ChannelClock);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET); // enable chip

	Parameter_update();
}

//methods for set************************************************
void extern_driver::SetDirection(dir direction){
	//Direction = direction;
	settings->Direct = direction;
}

void extern_driver::SetSpeed(uint16_t percent){
	if(percent >1000){percent = 1000;}
	if(percent <1){percent = 1;}
	settings->Speed = (uint16_t) map(percent, 1, 1000, MinSpeed, MaxSpeed);
	(TimFrequencies->Instance->ARR) = settings->Speed; // минимальная скорость
	Parameter_update();
}

void extern_driver::SetAcceleration(uint16_t percent){
	if(percent >1000){percent = 1000;}
	if(percent <1){percent = 1;}
	settings->Accel = (uint16_t) map(percent, 1, 1000, 1, settings->Speed);
	Parameter_update();
}

void extern_driver::SetDeacceleration(uint16_t percent){
	if(percent >1000){percent = 1000;}
	if(percent <1){percent = 1;}
	settings->Slowdown = percent;
	Parameter_update();

}

void extern_driver::SetCurrent(uint32_t mAmax){
	//CurrenrMax = mAmax;
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
	/*if(current < CurrenrSTOP){
		CurrenrMax = CurrenrSTOP;
	}else{
		CurrenrMax = current;
	}*/

}
void extern_driver::SetCurrentStop(unsigned int current){
	//CurrenrSTOP = current;
}
void extern_driver::SetPWRstatus(bool low){
	lowpwr = low;
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
	return (uint16_t) map(settings->Accel, 1, settings->Speed, 1, 1000);
}

uint32_t extern_driver::getSpeed() {

	return (uint32_t) map(settings->Speed, MinSpeed, MaxSpeed, 1, 1000);
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
bool extern_driver::start(){
	//   removeBreak(true);
	if (Status == statusMotor::STOPPED) {
		//Установка направления
		if (settings->Direct == dir::CCW) {

			HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
		}
		printf("Start motor.\r\n");
		(TimFrequencies->Instance->ARR) = settings->Speed; // минимальная скорость
		HAL_TIM_OC_Start(TimFrequencies, ChannelClock);
		return true;
	} else
	{
		printf("Fail started motor.\r\n");
		return false;
	}
}

void extern_driver::stop(){

	HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
	Status = statusMotor::STOPPED;
	printf("motion finished.\r\n");

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


//handlers*******************************************************
//счетчик операционный для счета шагов между операциями
void extern_driver::StepsHandler(uint32_t steps){

}

//счетчик обшего количества шагов
void extern_driver::StepsAllHandler(uint32_t steps){

}

void extern_driver::SensHandler(){


}

// обработчик таймера разгона торможения
void extern_driver::AccelHandler(){

}

//*******************************************************
void extern_driver::InitTim(){

}

extern_driver::extern_driver(){

}

extern_driver::extern_driver(TIM_HandleTypeDef *timFreq, uint32_t channelFreq) :

				TimFrequencies(timFreq), ChannelClock(channelFreq){

}
extern_driver::~extern_driver(){

}

double extern_driver::map(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
