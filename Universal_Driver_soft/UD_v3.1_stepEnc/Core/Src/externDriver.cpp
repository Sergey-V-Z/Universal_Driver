#include "motor.hpp"
#include "stdio.h"

/***************************************************************************
 * Класс для шагового двухфазного мотора
 *
 * В этом классе реализован цикл управления и контроля шагового двигателя
 ****************************************************************************/

void extern_driver::Init() {

	//settings = set;

	//Расчет максималных параметров PWM для скорости



	//установка делителя
	switch (settings->motor) {
		case motor_t::stepper_motor:

			TimFrequencies->Instance->PSC = 399; //(80 мГц/400) if MaxSpeed = 50 max 2 kHZ
			MaxSpeed = 50; //((TimFrequencies->Instance->ARR/100)*1);
			MinSpeed = 13000; //((TimFrequencies->Instance->ARR/100)*100);
			break;
		case motor_t::bldc:

			TimFrequencies->Instance->PSC = 200-1; //(80 мГц/400) if MaxSpeed = 100 max 4 kHZ
			MaxSpeed = 100; //((TimFrequencies->Instance->ARR/100)*1);
			MinSpeed = 2666; // ~150 Hz (80 мГц/200)/150 Hz = 2666
			break;
		default:
			break;
	}

	TimFrequencies->Instance->ARR = MinSpeed;
	//TimCountAllSteps->Instance->ARR = settings->Target;

	//установка скорости калибровки
	Speed_Call = (uint16_t) map(950, 1, 1000, MinSpeed, MaxSpeed); // 30% скорости

	Status = statusMotor::STOPPED;
	FeedbackType = fb::ENCODER; // сделать установку этого значения из настроек
	StatusTarget = statusTarget_t::finished;

	if (settings->Direct == dir::CW) {
		//HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
		DIRECT_CW
	} else if (settings->Direct == dir::CCW) {
		//HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
		DIRECT_CCW
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
    if (Status == statusMotor::STOPPED) {

        LastDistance = 0; // обнуляем счетчик предыдушего действия

        // Проверка концевиков через карту датчиков
        if (settings->sensors_map.detected) {
            // Проверяем не находимся ли мы на концевике, противоположном направлению движения
            if (settings->Direct == dir::CW &&
                HAL_GPIO_ReadPin(D0_GPIO_Port, settings->sensors_map.CW_sensor)) {
                STM_LOG("Cannot move CW: at CW limit switch");
                return false;
            }
            if (settings->Direct == dir::CCW &&
                HAL_GPIO_ReadPin(D1_GPIO_Port, settings->sensors_map.CCW_sensor)) {
                STM_LOG("Cannot move CCW: at CCW limit switch");
                return false;
            }
        }

        // Включаем игнорирование датчиков на время START_VIBRATION_TIMEOUT
        ignore_sensors = true;
        vibration_start_time = HAL_GetTick();

        LastDistance = 0; // обнуляем счетчик предыдушего действия

        //Установка направления

        // режим счетчика
        switch (settings->mod_rotation) {
			case mode_rotation_t::infinity:
			{

				break;
			}
			case mode_rotation_t::infinity_enc:
			{

				break;
			}
			case mode_rotation_t::by_meter_enc:
			{
		        if (settings->Direct == dir::CCW) {
		            DIRECT_CCW
					TimEncoder->Instance->CCR4 = settings->Target;
		            TimEncoder->Instance->CCR3 = (TimEncoder->Instance->CCR4) - settings->SlowdownDistance;
		            TimEncoder->Instance->CNT = 0;
		        } else {
		            DIRECT_CW
					TimEncoder->Instance->CCR4 = 0xffff - settings->Target;
		            TimEncoder->Instance->CCR3 = (TimEncoder->Instance->CCR4) + settings->SlowdownDistance;
		            TimEncoder->Instance->CNT = 0xffff;
		        }
				break;
			}
			case mode_rotation_t::by_meter_timer:
			{
		        if (settings->Direct == dir::CCW) {
		            DIRECT_CCW

		        } else {
		            DIRECT_CW
		        }

		        TimCountAllSteps->Instance->CNT = 0;
		        TimCountAllSteps->Instance->ARR = settings->Target - settings->SlowdownDistance;
				break;
			}
			case mode_rotation_t::by_meter_timer_limit_switch:
			{

		        if (settings->Direct == dir::CCW) {
		            DIRECT_CCW

		        } else {
		            DIRECT_CW
		        }

		        // если калибровка то выставить максимум шагов иначе калиброванные шаги минус торможение
				if(permission_calibrate)
				{
			        TimCountAllSteps->Instance->CNT = 0;
			        TimCountAllSteps->Instance->ARR = 0xffff;
				}
				else {
					// если калиброванные шаги не равны нулю и калибровка завершена и режим остановки по концевикам то использовать как таргет калиброванное значение
					if((settings->sensors_map.detected == true) && (CallSteps != 0))
					{
				        TimCountAllSteps->Instance->CNT = 0;
				        TimCountAllSteps->Instance->ARR = CallSteps - settings->SlowdownDistance;
					}
				}

				break;
			}
			default:
			{
				break;
			}
		}

        PrevCounterENC = TimEncoder->Instance->CNT;
        countErrDir = 3;
        StatusTarget = statusTarget_t::inProgress;

        switch (settings->motor) {
            case motor_t::stepper_motor:
                (TimFrequencies->Instance->ARR) = settings->StartSpeed;
                Status = statusMotor::ACCEL;
                break;
            case motor_t::bldc:
                (TimFrequencies->Instance->ARR) = settings->Speed;
                Status = statusMotor::MOTION;
                break;
            default:
                break;
        }

        STM_LOG("Start motor.");

        HAL_TIM_OC_Start(TimFrequencies, ChannelClock);
        return true;
    } else {
        STM_LOG("Fail started motor.");
        return false;
    }
}
/*
bool extern_driver::start() {
	//   removeBreak(true);
	if (Status == statusMotor::STOPPED) {

		//uint32_t centr = 32767;
		LastDistance = 0; // обнуляем счетчик предыдушего действия

		//Установка направления
		if (settings->Direct == dir::CCW) {

			//HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
			DIRECT_CCW
			TimEncoder->Instance->CCR4 = settings->Target; // прерывание по переполнению
			TimEncoder->Instance->CCR3 = (TimEncoder->Instance->CCR4) - settings->SlowdownDistance; // когда вызвать прерывание для начала торможения
			TimEncoder->Instance->CNT = 0;
		} else {
			//HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);
			DIRECT_CW
			TimEncoder->Instance->CCR4 = 0xffff - settings->Target; // прерывание по переполнению
			TimEncoder->Instance->CCR3 = (TimEncoder->Instance->CCR4) + settings->SlowdownDistance; // когда вызвать прерывание для начала торможения
			TimEncoder->Instance->CNT = 0xffff;

		}

		TimCountAllSteps->Instance->CNT = 0;
		TimCountAllSteps->Instance->ARR = settings->Target - settings->SlowdownDistance; // счет до торможения

		PrevCounterENC = TimEncoder->Instance->CNT;
		countErrDir = 3;
		StatusTarget = statusTarget_t::inProgress;

		switch (settings->motor) {
			case motor_t::stepper_motor:
				(TimFrequencies->Instance->ARR) = settings->StartSpeed; // скорость
				Status = statusMotor::ACCEL;
				break;
			case motor_t::bldc:
				(TimFrequencies->Instance->ARR) = settings->Speed; // скорость
				Status = statusMotor::MOTION;
				break;
			default:
				break;
		}

		STM_LOG("Start motor.");

		HAL_TIM_OC_Start(TimFrequencies, ChannelClock);
		return true;
	} else
	{
		STM_LOG("Fail started motor.");
		//stop(statusTarget_t::errMotion);
		return false;
	}
}*/

bool extern_driver::startForCall(dir d) {
	STM_LOG("Motor status: %d", (int)Status);
	STM_LOG("Direction set: %s", d == dir::CW ? "CW" : "CCW");
	STM_LOG("Speed set: %d", (int)(TimFrequencies->Instance->ARR));

	// настроим и запустим двигатель
	settings->Direct = d;
	// установить временный таргет


	return start();
}

void extern_driver::stop(statusTarget_t status) {

    // Выключаем игнорирование датчиков
    ignore_sensors = false;

	HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
	Status = statusMotor::STOPPED;

	switch (status) {
	case statusTarget_t::finished:
		//STM_LOG("motion finished.");
		break;
	case statusTarget_t::errMotion:
		//STM_LOG("motion err.");
		break;
	case statusTarget_t::errDirection:
		//STM_LOG("motion err direction.");

		break;
	default:
		break;
	}

	StatusTarget = status;

	TimerIsStart = false;
	Time = 0;

	// проверить в какаом мы режиме. если в движении то записываем количество шагов если в торможении то прибавляем к уже имеющимся шагам
	if (settings->Direct == dir::CCW) {
		//TimEncoder->Instance->CNT = 0; // сбросим счетчик энкодера
		if(settings->mod_rotation == by_meter_enc)
		{
			LastDistance = TimEncoder->Instance->CNT; // сколько шагов прошли
		}
		else
		{
			LastDistance = motionSteps + TimCountAllSteps->Instance->CNT; // сколько шагов прошли
			motionSteps = 0;
		}
	} else {
		//TimEncoder->Instance->CNT = 0xffff; // сбросим счетчик энкодера
		if(settings->mod_rotation == by_meter_enc)
		{
			LastDistance = 0xffff - TimEncoder->Instance->CNT; // сколько шагов прошли
		}
		else
		{
			LastDistance = motionSteps + TimCountAllSteps->Instance->CNT; // сколько шагов прошли
			motionSteps = 0;
		}

	}

}

void extern_driver::slowdown() {

	switch (settings->motor) {
		case motor_t::stepper_motor:
			//STM_LOG("Slowdown.");
			switch (settings->mod_rotation) {
			case infinity_enc:
				Status = statusMotor::BRAKING;
				//STM_LOG("inf_enc mode");
				break;
			case infinity:
				Status = statusMotor::BRAKING;
				//STM_LOG("inf mode");
				break;
			case by_meter_timer_limit_switch:
			case by_meter_timer:
				Status = statusMotor::BRAKING;
				//STM_LOG("CNT = %d.", (int)TimCountAllSteps->Instance->CNT);
				//STM_LOG("ARR = %d.", (int)TimCountAllSteps->Instance->ARR);
				// сохранить шаги
				motionSteps = TimCountAllSteps->Instance->CNT;
				TimCountAllSteps->Instance->CNT = 0;
				TimCountAllSteps->Instance->ARR = settings->SlowdownDistance; // считаем до остановки
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
			break;
		case motor_t::bldc:
			//STM_LOG("Stop.");
			stop(statusTarget_t::finished);
			break;
		default:
			//STM_LOG("Stop.");
			stop(statusTarget_t::finished);
			break;
	}

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

	switch (settings->mod_rotation) {
	case infinity_enc:

		break;
	case infinity:

		break;
	case by_meter_timer:
		switch (Status) {
			case statusMotor::ACCEL:
			case statusMotor::MOTION:
			{
				//STM_LOG("StepsAllHandler, steps: %d", steps);
				slowdown();
				break;
			}
			case statusMotor::BRAKING:
			{
				//STM_LOG("stoped mode by_meter_timer");
				stop(statusTarget_t::finished);
				break;
			}
			case statusMotor::STOPPED:
			{
				//STM_LOG("err from StepsAllHandler():statusMotor::STOPPED, motor stoped");
				stop(statusTarget_t::errDirection);
				break;
			}
			default:
			{
				//STM_LOG("err from StepsAllHandler():default, motor stoped");
				stop(statusTarget_t::errMotion);
				break;
			}
			//return;
		}

		break;
	case by_meter_timer_limit_switch:
		switch (Status) {
			case statusMotor::ACCEL:
			case statusMotor::MOTION:
			{
				//STM_LOG("StepsAllHandler, steps: %d", steps);
                if (!permission_calibrate) {
                     slowdown();
                 }
				//запускаем таймер и останавливаемся по концевику
				TimerIsStart = true;
				break;
			}
			case statusMotor::BRAKING:
			{
				//STM_LOG("stoped mode by_meter_timer");
				//stop(statusTarget_t::finished);
				// произошло прерывание таймера счетчика шагов но мы не останавливаемся

				break;
			}
			case statusMotor::STOPPED:
			{
				//STM_LOG("err from StepsAllHandler():statusMotor::STOPPED, motor stoped");
				//stop(statusTarget_t::errDirection);
				break;
			}
			default:
			{
				//STM_LOG("err from StepsAllHandler():default, motor stoped");
				// непредвиденное прерывания остановка
				stop(statusTarget_t::errMotion);
				break;
			}
			//return;
		}
		break;
	case by_meter_enc:

		break;
	default:

		break;
	}
}

void extern_driver::SensHandler(uint16_t GPIO_Pin) {

    // Если включено игнорирование датчиков, проверяем не истек ли таймаут
    if (ignore_sensors) {
        if ((HAL_GetTick() - vibration_start_time) < START_VIBRATION_TIMEOUT) {
            // Игнорируем прерывание
            return;
        } else {
            // Таймаут истек, выключаем игнорирование
            ignore_sensors = false;
        }
    }

    // Сохраняем последний сработавший датчик
    last_triggered_sensor = GPIO_Pin;

    stop(statusTarget_t::finished);

    if (settings->sensors_map.detected) {
        if (settings->Direct == dir::CW) {
            position = pos_t::D0;
        } else {
            position = pos_t::D1;
        }
    } else {
        position = pos_t::D_0_1;
    }
}
/*
void extern_driver::SensHandler(uint16_t GPIO_Pin) {

	// запустить таймер антидребезга
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

}*/

// обработчик разгона торможения
void extern_driver::AccelHandler() {
	switch (Status) {
	case statusMotor::ACCEL: {
        if (permission_calibrate) {
            // Если это калибровка, разгоняемся до Speed_Call
            if (((TimFrequencies->Instance->ARR) - settings->Accel) >= Speed_Call) {
                (TimFrequencies->Instance->ARR) -= settings->Accel;
            } else {
                (TimFrequencies->Instance->ARR) = Speed_Call;
                Status = statusMotor::MOTION;
            }
        } else {
            // Обычный разгон до settings->Speed
            if (((TimFrequencies->Instance->ARR) - settings->Accel) >= settings->Speed) {
                (TimFrequencies->Instance->ARR) -= settings->Accel;
            } else {
                (TimFrequencies->Instance->ARR) = settings->Speed;
                Status = statusMotor::MOTION;
            }
        }
		break;
	}
	case statusMotor::BRAKING: {
		if ((TimFrequencies->Instance->ARR) + settings->Slowdown <= MinSpeed) // если "торможение" больше или ровно минимальному то выставить минимум и остоновить торможение
			(TimFrequencies->Instance->ARR) += settings->Slowdown;
		else {
			switch (settings->mod_rotation) {
			case infinity:
				stop(statusTarget_t::finished);
				break;
			case infinity_enc:
				stop(statusTarget_t::finished);
				break;
			case by_meter_timer_limit_switch:
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


	switch (settings->mod_rotation) {
	case infinity:
		break;
	case infinity_enc:
		break;
	case by_meter_timer:
		break;
	case by_meter_timer_limit_switch:
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
				TimerIsStart = false;
				Time = 0;
				stop(statusTarget_t::errMotion); // передаем ноль останов по таймауту
			}
		}
		break;
	}

}

// Калибровка
bool extern_driver::Calibration_pool() {
    if (permission_calibrate) {
        // Проверяем текущее состояние датчиков
        bool on_D0 = HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin) == GPIO_PIN_SET;
        bool on_D1 = HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin) == GPIO_PIN_SET;

        STM_LOG("Starting calibration. D0: %d, D1: %d", on_D0, on_D1);

        if(on_D0) {
            // Если мы на D0, движемся к D1
            STM_LOG("On D0 sensor, moving to D1");
            startForCall(dir::CCW);

            for(;;) {
                if(Status == statusMotor::STOPPED) {
                    if(StatusTarget == statusTarget_t::finished) {
                        if(HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin) == GPIO_PIN_SET) {
                            STM_LOG("Successfully reached D1");
                            // Теперь двигаемся обратно к D0 для измерения расстояния
                            startForCall(dir::CW);

                            for(;;) {
                                if(Status == statusMotor::STOPPED) {
                                    if(StatusTarget == statusTarget_t::finished) {
                                        if(HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin) == GPIO_PIN_SET) {
                                            CallSteps = LastDistance;
                                            settings->sensors_map.detected = true;
                                            permission_calibrate = false;
                                            STM_LOG("Calibration completed. Steps: %d", CallSteps);
                                            return true;
                                        }
                                    }
                                    break;
                                }
                                osDelay(1);
                            }
                        }
                    }
                    break;
                }
                osDelay(1);
            }
        } else if(on_D1) {
            // Если мы на D1, движемся к D0 для завершения
            STM_LOG("On D1 sensor, moving to D0");
            startForCall(dir::CW);

            for(;;) {
                if(Status == statusMotor::STOPPED) {
                    if(StatusTarget == statusTarget_t::finished) {
                        if(HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin) == GPIO_PIN_SET) {
                            CallSteps = LastDistance;
                            settings->sensors_map.detected = true;
                            permission_calibrate = false;
                            STM_LOG("Calibration completed. Steps: %d", CallSteps);
                            return true;
                        }
                    }
                    break;
                }
                osDelay(1);
            }
        } else {
            // Если мы между датчиками, сначала движемся к D1
            STM_LOG("Between sensors, moving to D1");
            startForCall(dir::CCW);
        }
    }
    return false;
}

/*void extern_driver::Calibration_pool() {
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
*/
void extern_driver::findHome() {
	if (permission_findHome) {
		permission_findHome = false;

		// Проверяем наличие концевиков через карту датчиков
		if (!settings->sensors_map.detected) {
			STM_LOG("No limit switches detected in system");
			return;
		}

		// Определяем текущее положение
		if (HAL_GPIO_ReadPin(D1_GPIO_Port, settings->sensors_map.CCW_sensor)
				== GPIO_PIN_SET) {
			// Мы на CCW концевике
			STM_LOG("Starting from CCW sensor, moving CW");

			// Устанавливаем направление на CW
			SetDirection(dir::CW);


			// Запускаем движение
			start();

			// Ждем остановки мотора
			while (Status != statusMotor::STOPPED) {
				osDelay(1);
			}


		} else if (HAL_GPIO_ReadPin(D0_GPIO_Port,
				settings->sensors_map.CW_sensor) == GPIO_PIN_SET) {
			// Мы уже в домашней позиции (CW sensor)
			STM_LOG("Already at home position (CW sensor)");
			return;

		} else {
			// Мы между концевиками
			STM_LOG("Between sensors, moving to CCW first");

			// Двигаемся к CCW концевику
			SetDirection(dir::CCW);
			start();

			// Ждем достижения концевика
			while (Status != statusMotor::STOPPED) {
				osDelay(1);
			}

			if (StatusTarget != statusTarget_t::finished) {
				STM_LOG("Failed to reach CCW sensor");
				return;
			}

			// Теперь двигаемся к домашней позиции (CW)
			STM_LOG("Reached CCW sensor, moving to home position (CW)");
			SetDirection(dir::CW);
			start();

			// Ждем достижения домашней позиции
			while (Status != statusMotor::STOPPED) {
				osDelay(1);
			}

		}

		// Проверяем успешность достижения домашней позиции
		if (HAL_GPIO_ReadPin(D0_GPIO_Port, settings->sensors_map.CW_sensor)
				== GPIO_PIN_SET) {
			STM_LOG("Successfully reached home position");
		} else {
			STM_LOG("Failed to reach home position");
		}
	}
}

void extern_driver::findHomeStart()
{
	permission_findHome= true;
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
	settings->Speed = (uint32_t) map(percent, 1, 1000, MinSpeed, MaxSpeed);
	(TimFrequencies->Instance->ARR) = settings->Speed; // скорость
	if (Status == statusMotor::MOTION) {
		//(TimFrequencies->Instance->ARR) = settings->Speed; // скорость
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
	settings->StartSpeed = (uint32_t) map(percent, 1, 1000, MinSpeed, MaxSpeed);
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

void extern_driver::SetMode(mode_rotation_t mod) {
	settings->mod_rotation = mod;
}

void extern_driver::SetMotor(motor_t m) {
	if (m >= motor_t::stepper_motor && m <= motor_t::bldc)
		settings->motor = m;
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
 //return StartSpeed;
 return (uint32_t) map(settings->StartSpeed, MinSpeed, MaxSpeed, 1, 1000);
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
	return settings->mod_rotation;
}

motor_t extern_driver::getMotor() {
	return settings->motor;
}

statusTarget_t extern_driver::getStatusTarget() {
	return StatusTarget;
}

uint32_t extern_driver::getSlowdownDistance() {
	return settings->SlowdownDistance;
}

uint32_t extern_driver::getLastDistance() {
	return LastDistance;
}

void extern_driver::StartDebounceTimer(uint16_t GPIO_Pin) {
    if(GPIO_Pin == D0_Pin && !d0_debounce_active) {
        d0_debounce_active = true;
        // Настраиваем и запускаем таймер
        __HAL_TIM_SET_COUNTER(debounceTimer, 0);
        HAL_TIM_Base_Start_IT(debounceTimer);
    }
    else if(GPIO_Pin == D1_Pin && !d1_debounce_active) {
        d1_debounce_active = true;
        // Настраиваем и запускаем таймер
        __HAL_TIM_SET_COUNTER(debounceTimer, 0);
        HAL_TIM_Base_Start_IT(debounceTimer);
    }
}

void extern_driver::HandleDebounceTimeout() {
    // Останавливаем таймер
    HAL_TIM_Base_Stop_IT(debounceTimer);

    // Проверяем D0
    if(d0_debounce_active) {
        if(HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin) == GPIO_PIN_SET) {
            // Сигнал все еще активен - вызываем обработчик
            SensHandler(D0_Pin);
        }
        d0_debounce_active = false;
    }

    // Проверяем D1
    if(d1_debounce_active) {
        if(HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin) == GPIO_PIN_SET) {
            // Сигнал все еще активен - вызываем обработчик
            SensHandler(D1_Pin);
        }
        d1_debounce_active = false;
    }
}
//*******************************************************
void extern_driver::InitTim() {

}

double extern_driver::map(double x, double in_min, double in_max,
		double out_min, double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

extern_driver::~extern_driver() {

}

extern_driver::extern_driver(settings_t *set, TIM_HandleTypeDef *timCount,
		TIM_HandleTypeDef *timFreq, uint32_t channelFreq,
		TIM_HandleTypeDef *timDebounce, TIM_HandleTypeDef *timENC) :
		settings(set), TimCountAllSteps(timCount), TimFrequencies(timFreq), ChannelClock(
				channelFreq), debounceTimer(timDebounce), TimEncoder(timENC) {
}

