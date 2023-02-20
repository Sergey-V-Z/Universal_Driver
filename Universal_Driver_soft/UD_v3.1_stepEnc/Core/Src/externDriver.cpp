#include "motor.hpp"

/***************************************************************************
 * Класс для шагового двухфазного мотора
 *
 * В этом классе реализован цикл управления и контроля шагового двигателя
 ****************************************************************************/

void extern_driver::Init(settings_t *set){

	settings = set;

	//Расчет максималных параметров PWM для скорости
	MaxSpeed =  50;//((TimFrequencies->Instance->ARR/100)*1);
	MinSpeed =  10000;//((TimFrequencies->Instance->ARR/100)*100);

	//установка делителя
	TimFrequencies->Instance->PSC = 399; //(80 мГц/400)
	TimFrequencies->Instance->ARR = MinSpeed;
	TimCountAllSteps->Instance->ARR = settings->Target;

	//установка скорости калибровки
	Speed_Call = (uint16_t) map(15, 1, 1000, MinSpeed, MaxSpeed);

	Status = statusMotor::STOPPED;
	FeedbackType = fb::ENCODER; // сделать установку этого значения из настроек

	if(settings->Direct == dir::CW){
		HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
	}else if(settings->Direct == dir::CCW){
		HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
	}

	prevCounter = 65535/2;
	__HAL_TIM_SET_COUNTER(TimEncoder, prevCounter);
	__HAL_TIM_SET_COUNTER(TimCountAllSteps, 0);

	HAL_TIM_Base_Start_IT(TimCountAllSteps);
	HAL_TIM_Encoder_Start(TimEncoder, TIM_CHANNEL_ALL);

	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);



	Parameter_update();
}

//methods for set************************************************
void extern_driver::SetSpeed(uint16_t percent){
	if(percent >1000){percent = 1000;}
	if(percent <1){percent = 1;}
	settings->Speed = (uint16_t) map(percent, 1, 1000, MinSpeed, MaxSpeed);
	if(Status == statusMotor::MOTION){
		//TimFrequencies->Instance->CCR1 = Speed;
	}
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
	settings->Deaccel = percent;
	Parameter_update();

}

uint32_t extern_driver::SetTarget (uint32_t temp){
	if(temp >32000)
		temp = 32000;

	if(temp <1)
		temp = 1;

	settings->Target = temp;
	Parameter_update();
	return temp;
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

uint32_t extern_driver::getTarget() {
	return settings->Target;
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
	if(Status == statusMotor::STOPPED && StatusTarget == statusTarget_t::finished){

		//Установка направления
		if(settings->Direct == dir::CW){
			HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
		}

		TimAcceleration->Instance->CCR2 = 0;
		TimCountAllSteps->Instance->CNT = 0; //счетчик пульсов
		TimCountAllSteps->Instance->ARR = settings->Target;

		Status = statusMotor::ACCEL;
		StatusTarget = statusTarget_t::inProgress;
		HAL_TIM_OC_Start(TimFrequencies, ChannelClock);
		return true;
	}
	else
		return false;

}

void extern_driver::stop(){
	//   removeBreak(false);
	if((Status == statusMotor::MOTION) || (Status == statusMotor::BRAKING)){
		HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
		//(TimFrequencies->Instance->ARR) = MinSpeed;
		//TimCountAllSteps->Instance->ARR = 0;

		prevCounter = 65535/2;
		__HAL_TIM_SET_COUNTER(TimEncoder, prevCounter);

		Status = statusMotor::STOPPED;

	}
	StatusTarget = statusTarget_t::finished;
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

	HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
	Status = statusMotor::STOPPED;

	if(modCounter){
		int32_t error = 0;

		uint32_t currCounter = __HAL_TIM_GET_COUNTER(TimEncoder);
		/*
		currCounter = 32767 - ((currCounter-1) & 0xFFFF);
		if(currCounter > 32768/2) {
			// Преобразуем значения счетчика из:
			//  ... 32766, 32767, 0, 1, 2 ...
			// в значения:
			//  ... -2, -1, 0, 1, 2 ...
			currCounter = currCounter - 32768;
		}
		 */
		if(currCounter != prevCounter) {
			int32_t delta = currCounter-prevCounter;
			//prevCounter = currCounter;
			prevCounter = 65535/2;
			__HAL_TIM_SET_COUNTER(TimEncoder, prevCounter);

			if(delta != 0) {

				//Вычислить ошибку
				error = abs(delta) - TimCountAllSteps->Instance->ARR;
				//при делителе шага 20 количесво нагов на оборот 4000 и количестве шагов на оборот у энкодера 4000

				if((error >= -3) && (error <= 3)){error = 0;}

				if(error < 0){ // не доехали
					// перенастроить счетчик шагов
					TimCountAllSteps->Instance->ARR = abs(error);
					Status = statusMotor::ACCEL;
					HAL_TIM_OC_Start(TimFrequencies, ChannelClock);

				} else if(error > 0){ // переехали
					// сменим направление
					if(settings->Direct == dir::CCW)
						HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
					else
						if(settings->Direct == dir::CW)
							HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);

					// перенастроить счетчик шагов
					TimCountAllSteps->Instance->ARR = abs(error);
					Status = statusMotor::ACCEL;
					HAL_TIM_OC_Start(TimFrequencies, ChannelClock);

				} else
					if(error == 0){ // мы на месте
						this->stop();
					}
				// ...
			}
		}
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
		if((TimFrequencies->Instance->ARR) > settings->Speed){ // если "ускорение" меньше или ровно максимальному то выставить максимум
			//если разница меньше нуля
			if(TimFrequencies->Instance->ARR < settings->Accel){
				(TimFrequencies->Instance->ARR) = settings->Speed;
			}else{
				(TimFrequencies->Instance->ARR) -= settings->Accel; // Ускоряем
			}

		}else{
			(TimFrequencies->Instance->ARR) = settings->Speed;
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

extern_driver::extern_driver(TIM_HandleTypeDef *timCount, TIM_HandleTypeDef *timFreq, uint32_t channelFreq, TIM_HandleTypeDef *timAccel, TIM_HandleTypeDef *timENC) :
																TimCountAllSteps(timCount), TimFrequencies(timFreq), ChannelClock(channelFreq), TimAcceleration(timAccel), TimEncoder(timENC)
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

bool extern_driver::getStatusTarget() {
	if(StatusTarget == statusTarget_t::finished)
		return false;
	else
		return true;

}
