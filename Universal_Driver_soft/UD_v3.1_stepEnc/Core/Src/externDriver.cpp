#include "motor.hpp"
#include "stdio.h"

/***************************************************************************
 * Класс для шагового двухфазного мотора
 *
 * В этом классе реализован цикл управления и контроля шагового двигателя
 ****************************************************************************/

void extern_driver::Init(settings_t *set) {

	settings = set;

	//Расчет максималных параметров PWM для скорости
	MaxSpeed = 50; //((TimFrequencies->Instance->ARR/100)*1);
	MinSpeed = 13000; //((TimFrequencies->Instance->ARR/100)*100);

	//установка делителя
	TimFrequencies->Instance->PSC = 399; //(80 мГц/400)
	TimFrequencies->Instance->ARR = MinSpeed;
	//TimCountAllSteps->Instance->ARR = settings->Target;

	//установка скорости калибровки
	Speed_Call = (uint16_t) map(15, 1, 1000, MinSpeed, MaxSpeed); // 1.5% скорости

	Status = statusMotor::STOPPED;
	FeedbackType = fb::ENCODER; // сделать установку этого значения из настроек
	StatusTarget = statusTarget_t::finished;

	if (settings->Direct == dir::CW) {
		HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
	} else if (settings->Direct == dir::CCW) {
		HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
	}

	//настройки ускорения

	//таймер энкодера
	__HAL_TIM_SET_COUNTER(TimCountAllSteps, 0);

	//HAL_TIM_Base_Start_IT(TimCountAllSteps);				// Внутренний счетчик выданных шагов
	HAL_TIM_Encoder_Start(TimEncoder, TIM_CHANNEL_ALL); 	// Режим Энкодера
	HAL_TIM_OC_Start_IT(TimEncoder, TIM_CHANNEL_3);	// Для прерываний по 3 каналу сравнения
	HAL_TIM_OC_Start_IT(TimEncoder, TIM_CHANNEL_4);	// Для прерываний по 4 каналу сравнения
	HAL_TIM_Base_Start_IT(TimEncoder);		// Для прерываняй по переполнению

	__HAL_TIM_SET_COUNTER(TimCountAllSteps, 0);
	HAL_TIM_Base_Start_IT(TimCountAllSteps);

	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);

	Parameter_update();
}

//methods for aktion*********************************************
bool extern_driver::start() {
	//   removeBreak(true);
	if (Status == statusMotor::STOPPED) {

		//uint32_t centr = 32767;
		LastDistance = 0; // обнуляем счетчик предыдушего действия

		//Установка направления
		if (settings->Direct == dir::CCW) {

			HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
			TimEncoder->Instance->CCR4 = settings->Target; // прерывание по переполнению
			TimEncoder->Instance->CCR3 = (TimEncoder->Instance->CCR4) - settings->SlowdownDistance; // когда вызвать прерывание для начала торможения
			TimEncoder->Instance->CNT = 0;
		} else {
			HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
			TimEncoder->Instance->CCR4 = 0xffff - settings->Target; // прерывание по переполнению
			TimEncoder->Instance->CCR3 = (TimEncoder->Instance->CCR4) + settings->SlowdownDistance; // когда вызвать прерывание для начала торможения
			TimEncoder->Instance->CNT = 0xffff;

		}

		TimCountAllSteps->Instance->CNT = 0;
		TimCountAllSteps->Instance->ARR = settings->Target - settings->SlowdownDistance; // счет до торможения

		PrevCounterENC = TimEncoder->Instance->CNT;
		countErrDir = 3;
		Status = statusMotor::ACCEL;
		StatusTarget = statusTarget_t::inProgress;

		printf("Start motor.\r\n");
		(TimFrequencies->Instance->ARR) = StartSpeed; // минимальная скорость
		HAL_TIM_OC_Start(TimFrequencies, ChannelClock);
		return true;
	} else
	{
		printf("Fail started motor.\r\n");
		return false;
	}


}

bool extern_driver::startForCall(dir d) {
	if (Status == statusMotor::STOPPED) {
		// настроим и запустим двигатель
		LastDistance = 0; // обнуляем счетчик предыдушего действия
		settings->Direct = d;

		// проверка напрвления

		// если мы на position = pos_t::D0 и команда тудаже то отменить запуск
		if (HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin) && (d == dir::CW)) {
			return false;
		}

		// если мы на position = pos_t::D1 и команда тудаже то отменить запуск
		if (HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin) && (d == dir::CCW)) {
			return false;
		}

		//Установка направления
		if (settings->Direct == dir::CCW) {
			HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
			TimEncoder->Instance->CNT = 0; // сбросим счетчик энкодера
			TimEncoder->Instance->CCR3 = 0xffff;
			TimEncoder->Instance->CCR4 = 0xffff;
		} else {
			HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
			TimEncoder->Instance->CNT = 0xffff; // сбросим счетчик энкодера
			TimEncoder->Instance->CCR3 = 0;
			TimEncoder->Instance->CCR4 = 0;
		}

		PrevCounterENC = TimEncoder->Instance->CNT;
		countErrDir = 3;
		Status = statusMotor::ACCEL;
		StatusTarget = statusTarget_t::inProgress;
		(TimFrequencies->Instance->ARR) = MinSpeed; // минимальная скорость
		HAL_TIM_OC_Start(TimFrequencies, ChannelClock);
		return true;
	} else
		return false;
}

void extern_driver::stop(statusTarget_t status) {

	HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
	Status = statusMotor::STOPPED;

	switch (status) {
	case statusTarget_t::finished:
		printf("motion finished.\r\n");
		break;
	case statusTarget_t::errMotion:
		printf("motion err.\r\n");
		break;
	case statusTarget_t::errDirection:
		printf("motion err direction.\r\n");

		break;
	default:
		break;
	}

	StatusTarget = status;

	TimerIsStart = false;
	Time = 0;
	if (settings->Direct == dir::CCW) {
		//TimEncoder->Instance->CNT = 0; // сбросим счетчик энкодера
		LastDistance = TimEncoder->Instance->CNT; // сколько шагов прошли
	} else {
		//TimEncoder->Instance->CNT = 0xffff; // сбросим счетчик энкодера
		LastDistance = 0xffff - TimEncoder->Instance->CNT; // сколько шагов прошли
	}

}

void extern_driver::slowdown() {

	printf("Slowdown.\r\n");
	switch (mod_rotation) {
	case infinity_enc:
		Status = statusMotor::BRAKING;
		printf("inf_enc mode\r\n");
		break;
	case infinity:
		Status = statusMotor::BRAKING;
		printf("inf mode\r\n");
		break;
	case by_meter_timer:
		if(Status == statusMotor::BRAKING)
		{
			stop(statusTarget_t::finished);
			//printf("CNT = %d.\r\n", (int)TimCountAllSteps->Instance->CNT);
			printf("ARR = %d.\r\n", (int)TimCountAllSteps->Instance->ARR);
		}
		else if((Status == statusMotor::MOTION) || (Status == statusMotor::ACCEL))
		{
				Status = statusMotor::BRAKING;
				//printf("CNT = %d.\r\n", (int)TimCountAllSteps->Instance->CNT);
				printf("ARR = %d.\r\n", (int)TimCountAllSteps->Instance->ARR);
				TimCountAllSteps->Instance->CNT = 0;
				TimCountAllSteps->Instance->ARR = settings->SlowdownDistance; // считаем до остановки

		}

		break;
	case by_meter_enc:
	default:
		if ((Status == statusMotor::MOTION) || (Status == statusMotor::ACCEL) || (Status == statusMotor::BRAKING)) {
			if (settings->Direct == dir::CCW) {
				if (TimEncoder->Instance->CNT >= TimEncoder->Instance->CCR3 && TimEncoder->Instance->CNT <= TimEncoder->Instance->CCR4) {
					Status = statusMotor::BRAKING;
				}
				if (TimEncoder->Instance->CNT >= TimEncoder->Instance->CCR4) {
					stop(statusTarget_t::finished);
				}
			} else {
				if (TimEncoder->Instance->CNT <= TimEncoder->Instance->CCR3 && TimEncoder->Instance->CNT >= TimEncoder->Instance->CCR4) {
					Status = statusMotor::BRAKING;
				}
				if (TimEncoder->Instance->CNT <= TimEncoder->Instance->CCR4) {
					stop(statusTarget_t::finished);
				}
			}

		}
		break;
	}


	/*
	 //выяснить в честь чего прерывание
	 if(settings->Direct == dir::CCW){

	 if(TimEncoder->Instance->CNT <= 32767 - 10)
	 this->stop();
	 else
	 if(TimEncoder->Instance->CNT >= (32767 + settings->Target) - settings->Slowdown){
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
	 if(TimEncoder->Instance->CNT <= (32767 - settings->Target) + settings->Slowdown){
	 if((this->Status == statusMotor::MOTION) || (this->Status == statusMotor::ACCEL)){
	 this->Status = statusMotor::BRAKING;
	 TimEncoder->Instance->CCR3 = (32767 - settings->Target);
	 } else
	 if(this->Status == statusMotor::BRAKING)
	 this->stop();
	 }
	 }
	 */
}

void extern_driver::removeBreak(bool status) {
	if (status) {
		//      HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_SET);
	} else {
		//      HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_RESET);
	}
}

void extern_driver::goTo(int steps, dir direct) {

}

//handlers*******************************************************
//счетчик операционный для счета шагов между операциями
void extern_driver::StepsHandler(uint32_t steps) {

}

//счетчик обшего количества шагов
void extern_driver::StepsAllHandler(uint32_t steps) {

	if (Status == STOPPED)
	{
		return;
	}

	switch (mod_rotation) {
	case infinity_enc:

		break;
	case infinity:

		break;
	case by_meter_timer:
		printf("StepsAllHandler, steps: %d\r\n", steps);
		slowdown();
		break;
	case by_meter_enc:

		break;
	default:

		break;
	}
}

void extern_driver::SensHandler(uint16_t GPIO_Pin) {

	// при достижении коцевиков
	switch (GPIO_Pin) {
	case D0_Pin:
		stop(statusTarget_t::finished);
		// обновить позицию
		position = pos_t::D0;
		break;
	case D1_Pin:
		stop(statusTarget_t::finished);
		// обновить позицию
		position = pos_t::D1;
		break;
	default:
		break;
	}

}

// обработчик разгона торможения
void extern_driver::AccelHandler() {
	switch (Status) {
	case statusMotor::ACCEL: {
		//uint32_t
		//
		if (((TimFrequencies->Instance->ARR) - settings->Accel) >= settings->Speed) { // если "ускорение" меньше или ровно максимальному то выставить максимум
			(TimFrequencies->Instance->ARR) -= settings->Accel; // Ускоряем
		} else {
			(TimFrequencies->Instance->ARR) = settings->Speed;
			Status = statusMotor::MOTION;
		}
		break;
	}
	case statusMotor::BRAKING: {
		if ((TimFrequencies->Instance->ARR) + settings->Slowdown <= MinSpeed) // если "торможение" больше или ровно минимальному то выставить минимум и остоновить торможение
			(TimFrequencies->Instance->ARR) += settings->Slowdown;
		else {
			switch (mod_rotation) {
			case infinity:
				stop(statusTarget_t::finished);
				break;
			case infinity_enc:
				stop(statusTarget_t::finished);
				break;
			case by_meter_timer:
				(TimFrequencies->Instance->ARR) = MinSpeed;
				break;
			case by_meter_enc:
			default:
				(TimFrequencies->Instance->ARR) = MinSpeed;
				break;
			}

		}

		break;
	}
	default: {

		break;
	}
	}


	switch (mod_rotation) {
	case infinity:

		break;
	case infinity_enc:

		break;
	case by_meter_timer:

		break;
	case by_meter_enc:
	default:
		// проверка направления при разгоне
		if (Status == statusMotor::ACCEL) {

			if (PrevCounterENC != TimEncoder->Instance->CNT) {
				if (settings->Direct == dir::CCW) {
					if ((PrevCounterENC) > TimEncoder->Instance->CNT) {
						if (countErrDir == 0) {
							stop(statusTarget_t::errDirection);
						} else {
							countErrDir--;
						}
					}
				} else {
					if ((PrevCounterENC) < TimEncoder->Instance->CNT) {
						if (countErrDir == 0) {
							stop(statusTarget_t::errDirection);
						} else {
							countErrDir--;
						}
					}
				}

				PrevCounterENC = TimEncoder->Instance->CNT;
				TimerIsStart = false;
				Time = 0;
			} else {
				TimerIsStart = true;
			}
		}

		// проверка движения при движении или остановке
		if ((Status == statusMotor::MOTION) || (Status == statusMotor::BRAKING)) {
			// проверять энкодер если нет движения при статусе MOTION или ACCEL или BRAKING то запускаем таймер значение которого установленно пользователем

			if (((PrevCounterENC + 100) >= TimEncoder->Instance->CNT)
					&& (TimEncoder->Instance->CNT >= (PrevCounterENC - 100))) {
				TimerIsStart = true;
			} else {
				PrevCounterENC = TimEncoder->Instance->CNT;
				TimerIsStart = false;
				Time = 0;
			}
		}
		// по завершении таймаута останавливаем систеу устанавливаем флаг ошибки в StatusTarget
		// работа таймера
		if (TimerIsStart) {
			Time++;
			if (Time >= settings->TimeOut) {
				stop(statusTarget_t::errMotion); // передаем ноль останов по таймауту
			}
		}
		break;
	}

}

// Калибровка
void extern_driver::Calibration_pool() {
	if (permission_calibrate) {

		switch (position) {
		// начать движение в точку 1 и считать количество сделанных шагов
		case (pos_t::D0): {
			startForCall(dir::CCW);

			for (;;) {
				//ждем остановки
				if (Status == statusMotor::STOPPED) {
					//выясняем причину остановки
					if (StatusTarget == statusTarget_t::finished) { // мотор остановлен штатно

						if (position == pos_t::D1) { // планка на нужном датчике
							permission_calibrate = false; // отключаем калибровку
							CallSteps = LastDistance; //калибровка прошла успешно сохранияем значение
						} else { // планка не на нужном датчике
							permission_calibrate = false; // отключаем калибровку
							CallSteps = 0;
						}
					} else { // мотор остановлен не штатно
						permission_calibrate = false; // отключаем калибровку
						CallSteps = 0;
					}
					break; // выход из цикла
				}
				osDelay(1);
			}
			break; // выход из кейса
		}

			// если мы в точке 1 или гдето между точками то движемся в точку 0
		case (pos_t::D_0_1) ... (pos_t::D1): {
			startForCall(dir::CW); // начать движение в точку 1

			for (;;) {
				//ждем остановки
				if (Status == statusMotor::STOPPED) {
					//выясняем причину остановки
					if (StatusTarget == statusTarget_t::finished) { // мотор остановлен штатно

						if (position == pos_t::D0) { // планка на нужном датчике

						} else { // планка не на нужном датчике
							permission_calibrate = false; // отключаем калибровку
							CallSteps = 0;
						}
					} else { // мотор остановлен не штатно
						permission_calibrate = false; // отключаем калибровку
						CallSteps = 0;
					}
					break; // выход из цикла
				}
				osDelay(1);
			}
			break; // выход из кейса
		}

		default:
			break;
		}
	}
}

void extern_driver::CallStart() {
	permission_calibrate = true;
}

//methods for set************************************************
void extern_driver::SetSpeed(uint32_t percent) {
	if (percent > 1000) {
		percent = 1000;
	}
	if (percent < 1) {
		percent = 1;
	}
	settings->Speed = (uint16_t) map(percent, 1, 1000, MinSpeed, MaxSpeed);
	if (Status == statusMotor::MOTION) {
		//TimFrequencies->Instance->CCR1 = Speed;
	}
	Parameter_update();
}


void extern_driver::SetStartSpeed(uint32_t percent) {
	if (percent > 1000) {
		percent = 1000;
	}
	if (percent < 1) {
		percent = 1;
	}
	StartSpeed = (uint16_t) map(percent, 1, 1000, MinSpeed, MaxSpeed);
}

void extern_driver::SetAcceleration(uint32_t StepsINmS) {
	settings->Accel = StepsINmS;
	Parameter_update();
}

void extern_driver::SetSlowdown(uint32_t steps) {
	settings->Slowdown = steps;
	Parameter_update();

}

uint32_t extern_driver::SetTarget(uint32_t temp) {
	if (temp > 32766)
		temp = 32766;

	if (temp < 1)
		temp = 1;

	settings->Target = temp;
	Parameter_update();
	return temp;
}

void extern_driver::setTimeOut(uint32_t time) {
	settings->TimeOut = time;
}

void extern_driver::SetZeroPoint(void) {

}

void extern_driver::SetMode(uint32_t mod) {
	switch (mod) {
	case 1:
		mod_rotation = infinity_enc;
		break;
	case 2:
		mod_rotation = infinity;
		break;
	case 3:
		mod_rotation = by_meter_timer;
		break;
	case 4:
		mod_rotation = by_meter_enc;
		break;
	default:
		mod_rotation = by_meter_enc;
		break;
	}

}

// расчитывает и сохраняет все параметры разгона и торможения
void extern_driver::Parameter_update(void) {

	//FeedbackBraking_P1 = target - ((uint16_t) map(Deaccel, 1, 1000, 1, target));
	//FeedbackBraking_P0 = CircleCounts - ((uint16_t) map(Deaccel, 1, 1000, 1, CircleCounts - target)); // Начало торможения перед нулевой точкой в отсчетах это 1000

	//расчет шага ускорения/торможения
	//Accel = Speed / (target - FeedbackBraking_P1);
	//TimCountAllSteps->Instance->ARR = target;
}

void extern_driver::SetDirection(dir direction) {
	settings->Direct = direction;
}

void extern_driver::SetSlowdownDistance(uint32_t steps) {
	settings->SlowdownDistance = steps;
}

//methods for get************************************************

uint32_t extern_driver::getAcceleration() {
	return (uint32_t) settings->Accel;
}

uint32_t extern_driver::getSlowdown() {
	return (uint32_t) settings->Slowdown;
}

uint32_t extern_driver::getSpeed() {

	return (uint32_t) map(settings->Speed, MinSpeed, MaxSpeed, 1, 1000);
}

uint32_t extern_driver::getStartSpeed() {
 return StartSpeed;
}

uint32_t extern_driver::getTarget() {
	return settings->Target;
}

uint32_t extern_driver::getTimeOut() {
	return settings->TimeOut;
}
pos_t extern_driver::get_pos() {
	return position;
}

dir extern_driver::getStatusDirect() {
	return settings->Direct;
}

statusMotor extern_driver::getStatusRotation() {
	return Status;
}

uint16_t extern_driver::getRPM() {
	return 0;
}

mode_rotation_t extern_driver::getMode() {
	return mod_rotation;
}

uint8_t extern_driver::getStatusTarget() {
	return StatusTarget;
}

uint32_t extern_driver::getSlowdownDistance() {
	return settings->SlowdownDistance;
}

uint32_t extern_driver::getLastDistance() {
	return LastDistance;
}

//*******************************************************
void extern_driver::InitTim() {

}

double extern_driver::map(double x, double in_min, double in_max,
		double out_min, double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

extern_driver::extern_driver(TIM_HandleTypeDef *timCount,
		TIM_HandleTypeDef *timFreq, uint32_t channelFreq,
		TIM_HandleTypeDef *timAccel, TIM_HandleTypeDef *timENC) :
		TimCountAllSteps(timCount), TimFrequencies(timFreq), ChannelClock(
				channelFreq), TimAcceleration(timAccel), TimEncoder(timENC) {

}

extern_driver::~extern_driver() {

}



