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

	//настройки ускорения

	//таймер энкодера
	__HAL_TIM_SET_COUNTER(TimCountAllSteps, 0);

	//HAL_TIM_Base_Start_IT(TimCountAllSteps);				// Внутренний счетчик выданных шагов
	HAL_TIM_Encoder_Start(TimEncoder, TIM_CHANNEL_ALL); 	// Режим Энкодера
	HAL_TIM_OC_Start_IT(TimEncoder, TIM_CHANNEL_3);		// Для прерываний по 3 каналу сравнения
	HAL_TIM_OC_Start_IT(TimEncoder, TIM_CHANNEL_4);		// Для прерываний по 4 каналу сравнения
	HAL_TIM_Base_Start_IT(TimEncoder);					// Для прерываняй по переполнению

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
	settings->AccelPer = percent;
	Parameter_update();
}

void extern_driver::SetSlowdown(uint16_t percent){
	if(percent >1000){percent = 1000;}
	if(percent <1){percent = 1;}
	settings->SlowdownPer = percent;
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

void extern_driver::setTimeOut(uint32_t time) {
	settings->TimeOut = time;
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
void extern_driver::SetDirection(dir direction) {
	settings->Direct = direction;
}
//methods for get************************************************

uint32_t extern_driver::getAccelerationPer() {
	return (uint32_t) settings->AccelPer;
}

uint32_t extern_driver::getSlowdownPer() {
	return (uint32_t) settings->SlowdownPer;
}

uint32_t extern_driver::getSpeed() {

	return (uint32_t) map(settings->Speed, MinSpeed, MaxSpeed, 1, 1000);
}

uint32_t extern_driver::getTarget() {
	return settings->Target;
}

uint32_t extern_driver::getTimeOut() {
	return settings->TimeOut;
}
uint32_t extern_driver::get_pos(){
	return 0;
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

uint8_t extern_driver::getStatusTarget() {
	return StatusTarget;
}

//methods for aktion*********************************************
bool extern_driver::start(){
	//   removeBreak(true);
	if(Status == statusMotor::STOPPED && StatusTarget == statusTarget_t::finished){

		//Установка направления
		if(settings->Direct == dir::CCW){
			HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
			TimEncoder->Instance->CNT = 32767;
			this->Slowdown = (uint32_t) map(settings->SlowdownPer, 1, 1000, 1, settings->Target); //выставляем ускорение
			TimEncoder->Instance->CCR3 = (32767 + settings->Target) - Slowdown;
			TimEncoder->Instance->CCR4 = 32767 - 10;
		}
		else{
			HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
			TimEncoder->Instance->CNT = 32767;
			Slowdown = (uint32_t) map(settings->SlowdownPer, 1, 1000, 1, settings->Target); //выставляем ускорение
			TimEncoder->Instance->CCR3 = (32767 - settings->Target) + Slowdown;
			TimEncoder->Instance->CCR4 = 32767 + 10;
		}

		this->Status = statusMotor::ACCEL;
		this->StatusTarget = statusTarget_t::inProgress;
		this->Accel = (uint32_t) map(settings->AccelPer, 1, 1000, MinSpeed, settings->Speed); //выставляем ускорение
		this->Slowdown = (uint32_t) map(settings->SlowdownPer, 1, 1000, MinSpeed, settings->Speed); //выставляем замедление

		HAL_TIM_OC_Start(TimFrequencies, ChannelClock);
		return true;
	}
	else
		return false;

}

void extern_driver::stop(){
	HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
	this->Status = statusMotor::STOPPED;
	this->StatusTarget = statusTarget_t::finished;
}

void extern_driver::slowdown(){
	//выяснить в честь чего прерывание
	if(settings->Direct == dir::CCW){

		if(TimEncoder->Instance->CNT <= 32767 - 10)
			this->stop();
		else
			if(TimEncoder->Instance->CNT >= (32767 + settings->Target) - Slowdown){
				if((this->Status == statusMotor::MOTION) || (this->Status == statusMotor::ACCEL)){
					this->Status = statusMotor::BRAKING;
					TimEncoder->Instance->CCR3 = (32767 + settings->Target);
				} else
					if(this->Status == statusMotor::BRAKING)
						this->stop();
			}

	} else {

		if(TimEncoder->Instance->CNT >= 32767 - 10)
			this->stop();
		else
			if(TimEncoder->Instance->CNT <= (32767 - settings->Target) + Slowdown){
				if((this->Status == statusMotor::MOTION) || (this->Status == statusMotor::ACCEL)){
					this->Status = statusMotor::BRAKING;
					TimEncoder->Instance->CCR3 = (32767 - settings->Target);
				} else
					if(this->Status == statusMotor::BRAKING)
						this->stop();
			}
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
void extern_driver::StepsAllHandler(uint32_t parent){

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
			//
			if(TimFrequencies->Instance->ARR < Accel)
				(TimFrequencies->Instance->ARR) = settings->Speed;
			else
				(TimFrequencies->Instance->ARR) -= Accel; // Ускоряем
		} else {
			(TimFrequencies->Instance->ARR) = settings->Speed;
			Status = statusMotor::MOTION;
		}
		break;
	}
	case statusMotor::BRAKING:
	{
		if((TimFrequencies->Instance->ARR) < MinSpeed) // если "торможение" больше или ровно минимальному то выставить минимум и остоновить торможение
			(TimFrequencies->Instance->ARR) += Slowdown;
		else
			(TimFrequencies->Instance->ARR) = MinSpeed;

		break;
	}
	default:
	{
		break;
	}
	}
	// проверять энкодер если нет движения при статусе MOTION или ACCEL или BRAKING то запускаем маймер значение которого установленно пользователем
	// по завершении таймаута останавливаем систеу устанавливаем флаг ошибки в StatusTarget
}

void extern_driver::HandlerBrakingPoint() {
	this->slowdown();
}

void extern_driver::HandlerStop() {
	this->stop();
}

//*******************************************************
void extern_driver::InitTim(){

}

double extern_driver::map(double x, double in_min, double in_max,
		double out_min, double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

extern_driver::extern_driver(TIM_HandleTypeDef *timCount, TIM_HandleTypeDef *timFreq, uint32_t channelFreq, TIM_HandleTypeDef *timAccel, TIM_HandleTypeDef *timENC) :
					TimCountAllSteps(timCount), TimFrequencies(timFreq), ChannelClock(channelFreq), TimAcceleration(timAccel), TimEncoder(timENC)
{

}

extern_driver::~extern_driver(){

}

