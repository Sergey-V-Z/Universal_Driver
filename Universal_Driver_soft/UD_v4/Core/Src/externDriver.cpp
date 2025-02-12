#include "motor.hpp"
#include "stdio.h"

/***************************************************************************
 * Класс для шагового двухфазного мотора
 *
 * В этом классе реализован цикл управления и контроля шагового двигателя
 ****************************************************************************/

void extern_driver::Init() {
    currentDriverStatus = DRIVER_STATUS_UNKNOWN;
    lastDriverCheckTime = 0;
    driverErrorPort = DRIVER_ERR_GPIO_Port;
    driverErrorPin = DRIVER_ERR_Pin;

    switch (settings->motor) {
        case motor_t::stepper_motor:
            TimFrequencies->Instance->PSC = 399;
            MaxSpeed = 50;
            MinSpeed = 13000;
            break;
        case motor_t::bldc:
            TimFrequencies->Instance->PSC = 200-1;
            MaxSpeed = 100;
            MinSpeed = 2666;
            break;
        default:
            break;
    }

    TimFrequencies->Instance->ARR = MinSpeed;
    Speed_Call = (uint16_t) map(950, 1, 1000, MinSpeed, MaxSpeed);

    Status = statusMotor::STOPPED;
    FeedbackType = fb::ENCODER;
    StatusTarget = statusTarget_t::finished;

    if (settings->Direct == dir::CW) {
        DIRECT_CW
    } else if (settings->Direct == dir::CCW) {
        DIRECT_CCW
    }

    __HAL_TIM_SET_COUNTER(TimCountAllSteps, 0);
    HAL_TIM_Encoder_Start(TimEncoder, TIM_CHANNEL_ALL);
    HAL_TIM_OC_Start_IT(TimEncoder, TIM_CHANNEL_3);
    HAL_TIM_OC_Start_IT(TimEncoder, TIM_CHANNEL_4);
    HAL_TIM_Base_Start_IT(TimEncoder);

    __HAL_TIM_SET_COUNTER(TimCountAllSteps, 0);
    HAL_TIM_Base_Start_IT(TimCountAllSteps);

    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);

    checkDriverStatus();
    Parameter_update();
}

driver_status_t extern_driver::getDriverStatus() {
    if (!isDriverStatusValid()) {
        checkDriverStatus();
    }
    return currentDriverStatus;
}

uint32_t extern_driver::getLastDriverCheck() {
    return lastDriverCheckTime;
}

void extern_driver::checkDriverStatus() {
    GPIO_PinState errorState = HAL_GPIO_ReadPin(driverErrorPort, driverErrorPin);
    lastDriverCheckTime = HAL_GetTick();

    if (errorState == GPIO_PIN_SET) {
        currentDriverStatus = DRIVER_ERROR;
        if (Status != statusMotor::ERROR_M) {
            stop(statusTarget_t::errDriver);
            Status = statusMotor::ERROR_M;
        }
    } else {
        currentDriverStatus = DRIVER_OK;
    }
}

bool extern_driver::isDriverStatusValid() {
    return (HAL_GetTick() - lastDriverCheckTime) < DRIVER_STATUS_VALIDITY_TIME;
}

//methods for aktion*********************************************
bool extern_driver::start() {
    if (Status == statusMotor::STOPPED) {
        // Проверяем состояние драйвера перед стартом
        checkDriverStatus();
        if (currentDriverStatus != DRIVER_OK) {
            STM_LOG("Cannot start: driver error detected");
            return false;
        }

        LastDistance = 0;


		if (settings->Direct == dir::CW && HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin)) {
			STM_LOG("Cannot move CW: at CW limit switch");
			return false;
		}
		else if (settings->Direct == dir::CCW && HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin)) {
			STM_LOG("Cannot move CCW: at CCW limit switch");
			return false;
		} else {

		}


        ignore_sensors = true;
        vibration_start_time = HAL_GetTick();

        LastDistance = 0;

        switch (settings->mod_rotation) {
            case mode_rotation_t::infinity:
            case mode_rotation_t::infinity_enc:
                if (settings->Direct == dir::CCW) {
                    DIRECT_CCW
                } else {
                    DIRECT_CW
                }
                break;

            case mode_rotation_t::by_meter_enc:
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

            case mode_rotation_t::by_meter_timer:
                if (settings->Direct == dir::CCW) {
                    DIRECT_CCW
                } else {
                    DIRECT_CW
                }
                TimCountAllSteps->Instance->CNT = 0;
                TimCountAllSteps->Instance->ARR = settings->Target - settings->SlowdownDistance;
                break;

            case mode_rotation_t::by_meter_timer_limit_switch:
                if (settings->Direct == dir::CCW) {
                    DIRECT_CCW
                } else {
                    DIRECT_CW
                }
                if(permission_calibrate) {
                    TimCountAllSteps->Instance->CNT = 0;
                    TimCountAllSteps->Instance->ARR = 0xffff;
                } else {
                    if((settings->sensors_map.detected == true) && (CallSteps != 0)) {
                        TimCountAllSteps->Instance->CNT = 0;
                        TimCountAllSteps->Instance->ARR = CallSteps - settings->SlowdownDistance;
                    } else {
            			STM_LOG("Cannot move: not calibrated");
            			return false;
                    }
                }
                break;

            default:
                break;
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

        STM_LOG("Speed set: %d", (int)(TimFrequencies->Instance->ARR));
        STM_LOG("Start motor.");

        HAL_TIM_OC_Start(TimFrequencies, ChannelClock);

        return true;
    } else {
        STM_LOG("Fail started motor.");
        return false;
    }
}

void extern_driver::stop(statusTarget_t status) {
    ignore_sensors = false;

    HAL_TIM_OC_Stop(TimFrequencies, ChannelClock);
    //permission_calibrate = false;
    //permission_findHome = false;

    switch (status) {
        case statusTarget_t::finished:
            break;
        case statusTarget_t::errMotion:
            break;
        case statusTarget_t::errDirection:
            break;
        case statusTarget_t::errDriver:
            STM_LOG("Motor stopped: driver error detected");
            break;
        default:
            break;
    }

    StatusTarget = status;
    TimerIsStart = false;
    Time = 0;

    if (settings->Direct == dir::CCW) {
        if(settings->mod_rotation == by_meter_enc) {
            LastDistance = TimEncoder->Instance->CNT;
        } else {
            LastDistance = motionSteps + TimCountAllSteps->Instance->CNT;
            motionSteps = 0;
        }
    } else {
        if(settings->mod_rotation == by_meter_enc) {
            LastDistance = 0xffff - TimEncoder->Instance->CNT;
        } else {
            LastDistance = motionSteps + TimCountAllSteps->Instance->CNT;
            motionSteps = 0;
        }
    }

    Status = statusMotor::STOPPED;
}

bool extern_driver::startForCall(dir d) {
	STM_LOG("Motor status: %d", (int)Status);
	STM_LOG("Direction set: %s", d == dir::CW ? "CW" : "CCW");


	// настроим и запустим двигатель
	settings->Direct = d;
	// установить временный таргет


	return start();
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
    if(Status != statusMotor::STOPPED) {
        updateMotionCounter();
    }

    switch (settings->mod_rotation) {
        case infinity:
        case infinity_enc:
            // Для бесконечных режимов просто считаем шаги
            break;

        case by_meter_timer:
            switch (Status) {
                case statusMotor::ACCEL:
                case statusMotor::MOTION:
                    slowdown();
                    break;
                case statusMotor::BRAKING:
                    stop(statusTarget_t::finished);
                    break;
                case statusMotor::STOPPED:
                    stop(statusTarget_t::errDirection);
                    break;
                default:
                    stop(statusTarget_t::errMotion);
                    break;
            }
            break;

        case by_meter_timer_intermediate:
        case by_meter_timer_limit_switch:
            if (settings->motor == motor_t::stepper_motor) {
                switch (Status) {
                    case statusMotor::ACCEL:
                    case statusMotor::MOTION: {
                        if (!permission_calibrate) {
                            // В режиме обычного движения проверяем необходимость торможения
                            if(abs((int32_t)settings->points.current_steps - (int32_t)settings->Target) <= settings->SlowdownDistance) {
                                slowdown();
                            }
                        }
                        TimerIsStart = true;
                        break;
                    }
                    case statusMotor::BRAKING: {
                        // Проверяем достижение целевой позиции
                        if(abs((int32_t)settings->points.current_steps - (int32_t)settings->Target) <= 1) {
                            stop(statusTarget_t::finished);
                        }
                        break;
                    }
                    case statusMotor::STOPPED: {
                        break;
                    }
                    default: {
                        stop(statusTarget_t::errMotion);
                        break;
                    }
                }
            } else {
                // Режим BLDC - остановка по концевику
            }
            break;

        case by_meter_enc:
        case by_meter_enc_intermediate:
            // Для режимов с энкодером проверка позиции и торможение
            // выполняются в updateMotionCounter()
            if(Status != statusMotor::STOPPED) {
                if(settings->Direct == dir::CCW) {
                    if(TimEncoder->Instance->CNT >= TimEncoder->Instance->CCR3 &&
                       TimEncoder->Instance->CNT <= TimEncoder->Instance->CCR4) {
                        slowdown();
                    }
                    if(TimEncoder->Instance->CNT >= TimEncoder->Instance->CCR4) {
                        stop(statusTarget_t::finished);
                    }
                } else {
                    if(TimEncoder->Instance->CNT <= TimEncoder->Instance->CCR3 &&
                       TimEncoder->Instance->CNT >= TimEncoder->Instance->CCR4) {
                        slowdown();
                    }
                    if(TimEncoder->Instance->CNT <= TimEncoder->Instance->CCR4) {
                        stop(statusTarget_t::finished);
                    }
                }
            }
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
            position = pos_t::D_0_1;
        }
    }

    // Сохраняем последний сработавший датчик
    //last_triggered_sensor = GPIO_Pin;

    stop(statusTarget_t::finished);

	if (GPIO_Pin == D0_Pin) {
		position = pos_t::D0;
        settings->points.current_steps = 0;
        settings->points.current_point = 0;
	}
	else if (GPIO_Pin == D1_Pin){
		position = pos_t::D1;
        settings->points.current_steps = settings->points.points[settings->points.count - 1];
        settings->points.current_point = settings->points.count - 1;
	}
	else {
		position = pos_t::D_0_1;
	}

}

/**
 * Обработчик разгона и торможения двигателя.
 * Управляет ускорением, замедлением и проверяет корректность движения в зависимости от режима работы.
 */
void extern_driver::AccelHandler() {
    switch (Status) {
        case statusMotor::ACCEL: {
            if (permission_calibrate) {
                // Режим калибровки - разгон до специальной скорости Speed_Call
                if (((TimFrequencies->Instance->ARR) - settings->Accel) >= settings->Speed) {
                    (TimFrequencies->Instance->ARR) -= settings->Accel;  // Плавное увеличение скорости
                } else {
                    (TimFrequencies->Instance->ARR) = settings->Speed;   // Достигнута целевая скорость
                    Status = statusMotor::MOTION;                        // Переход в режим движения
                }
            } else {
                // Обычный режим работы - разгон до настроенной скорости settings->Speed
                if (((TimFrequencies->Instance->ARR) - settings->Accel) >= settings->Speed) {
                    (TimFrequencies->Instance->ARR) -= settings->Accel;  // Плавное увеличение скорости
                } else {
                    (TimFrequencies->Instance->ARR) = settings->Speed;   // Достигнута целевая скорость
                    Status = statusMotor::MOTION;                        // Переход в режим движения
                }
            }
            break;
        }

        case statusMotor::BRAKING: {
            // Обработка торможения - плавное уменьшение скорости
            if ((TimFrequencies->Instance->ARR) + settings->Slowdown <= MinSpeed) {
                (TimFrequencies->Instance->ARR) += settings->Slowdown;   // Плавное уменьшение скорости
            } else {
                // Действия при достижении минимальной скорости зависят от режима работы
                switch (settings->mod_rotation) {
                    case infinity:
                    case infinity_enc:
                        // Для бесконечных режимов - полная остановка
                        stop(statusTarget_t::finished);
                        break;

                    case by_meter_timer:
                    case by_meter_enc:
                    case by_meter_timer_limit_switch:
                    case by_meter_enc_limit_switch:
                    case by_meter_timer_intermediate:
                    case by_meter_enc_intermediate:
                        // Для режимов с метражом - установка минимальной скорости
                        (TimFrequencies->Instance->ARR) = MinSpeed;
                        break;
                }
            }
            break;
        }

        default:
            break;
    }

    // Обработка специфики различных режимов работы
    switch (settings->mod_rotation) {
        case infinity:
            // Бесконечное вращение без энкодера
            // Не требует дополнительных проверок
            break;

        case infinity_enc:
            // Бесконечное вращение с энкодером
            // Проверяем только наличие движения
            checkEncoderMotion();
            break;

        case by_meter_timer:
            // Режим работы по счетчику с таймером
            // Проверяем только таймер
            if (TimerIsStart) {
                Time++;
                if (Time >= settings->TimeOut) {
                    TimerIsStart = false;
                    Time = 0;
                    stop(statusTarget_t::errMotion);
                }
            }
            break;

        case by_meter_enc:
        case by_meter_enc_limit_switch:
        case by_meter_enc_intermediate:
            // Режимы работы с энкодером
            // Полная проверка направления и движения
            if (Status == statusMotor::ACCEL) {
                checkEncoderDirection();  // Проверка правильности направления при разгоне
            }
            if ((Status == statusMotor::MOTION) || (Status == statusMotor::BRAKING)) {
                checkEncoderMotion();     // Проверка наличия движения
            }
            // Проверка таймера
            if (TimerIsStart) {
                Time++;
                if (Time >= settings->TimeOut) {
                    TimerIsStart = false;
                    Time = 0;
                    stop(statusTarget_t::errMotion);
                }
            }
            break;

        case by_meter_timer_limit_switch:
        case by_meter_timer_intermediate:
            // Режимы работы с таймером
            // Проверяем только таймер
            if (TimerIsStart) {
                Time++;
                if (Time >= settings->TimeOut) {
                    TimerIsStart = false;
                    Time = 0;
                    stop(statusTarget_t::errMotion);
                }
            }
            break;
    }
}

/**
 * Проверка правильности направления движения по энкодеру
 * Останавливает двигатель при обнаружении неверного направления
 */
void extern_driver::checkEncoderDirection() {
    if (PrevCounterENC != TimEncoder->Instance->CNT) {
        if (settings->Direct == dir::CCW) {
            // Проверка направления для против часовой стрелки
            if ((PrevCounterENC) > TimEncoder->Instance->CNT) {
                if (countErrDir == 0) {
                    stop(statusTarget_t::errDirection);  // Остановка при ошибке направления
                } else {
                    countErrDir--;                      // Уменьшение счетчика ошибок
                }
            }
        } else {
            // Проверка направления для по часовой стрелке
            if ((PrevCounterENC) < TimEncoder->Instance->CNT) {
                if (countErrDir == 0) {
                    stop(statusTarget_t::errDirection);  // Остановка при ошибке направления
                } else {
                    countErrDir--;                      // Уменьшение счетчика ошибок
                }
            }
        }

        PrevCounterENC = TimEncoder->Instance->CNT;    // Обновление предыдущего значения энкодера
        TimerIsStart = false;                          // Сброс таймера
        Time = 0;
    } else {
        TimerIsStart = true;                           // Запуск таймера если нет движения
    }
}

/**
 * Проверка наличия движения по энкодеру
 * Запускает таймер при отсутствии движения
 */
void extern_driver::checkEncoderMotion() {
    // Проверка изменения значения энкодера в допустимых пределах (+/- 100)
    if (((PrevCounterENC + 100) >= TimEncoder->Instance->CNT)
        && (TimEncoder->Instance->CNT >= (PrevCounterENC - 100))) {
        TimerIsStart = true;                           // Запуск таймера при отсутствии движения
    } else {
        PrevCounterENC = TimEncoder->Instance->CNT;    // Обновление предыдущего значения
        TimerIsStart = false;                          // Сброс таймера
        Time = 0;
    }
}

// Калибровка
bool extern_driver::Calibration_pool() {

	// доработать калибровку. если один из датчиков вышел из строя и мотор сделает
	// круг и снова поподет на тот же концевик от куда начал то он долже остановится

    if (permission_calibrate && (settings->motor == motor_t::stepper_motor)) {
        // Проверяем текущее состояние датчиков
        bool on_D0 = HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin) == GPIO_PIN_SET;
        bool on_D1 = HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin) == GPIO_PIN_SET;

        //STM_LOG("Starting calibration. D0: %d, D1: %d", on_D0, on_D1);

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
        } else if (Status == statusMotor::STOPPED){
            // Если мы между датчиками, сначала движемся к D1
            STM_LOG("Between sensors, moving to D1");
            startForCall(dir::CCW);
        } else {
        	// ошибка стоит прервать калибровку
        }
    }
    return false;
}


void extern_driver::findHome() {
	if (permission_findHome && (settings->motor == motor_t::stepper_motor)) {
		permission_findHome = false;

		// Проверяем наличие концевиков через карту датчиков
		if (!settings->sensors_map.detected) {
			STM_LOG("No limit switches detected in system");
			return;
		}

		// Определяем текущее положение
		if (HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin)
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


		} else if (HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin) == GPIO_PIN_SET) {
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
		if (HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin) == GPIO_PIN_SET) {
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
	else if (percent < 1) {
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
    if (!settings) {
        STM_LOG("Error: settings is null");
        return;
    }
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

	pos_t tmp_pos = pos_t::D_0_1;

	GPIO_PinState D0 = HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin);
	GPIO_PinState D1 = HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin);

	if ((D0 == GPIO_PIN_SET) && (D1 == GPIO_PIN_RESET)) {
		tmp_pos = pos_t::D0;
	}
	else if ((D0 == GPIO_PIN_RESET) && (D1 == GPIO_PIN_SET)){
		tmp_pos = pos_t::D1;
	}
	else {
		tmp_pos = pos_t::D_0_1;
	}

	return tmp_pos;
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
	// если мы находимся между концевиками то нужно остановить иначе запустить антидребезг
	if((position == pos_t::D_0_1) && (ignore_sensors == false))
	{
		SensHandler(GPIO_Pin);
	}
	else
	{
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

bool extern_driver::validatePointNumber(uint32_t point_number) {
    return (point_number < MAX_POINTS);
}

void extern_driver::updateCurrentSteps(int32_t steps) {
    if(settings->Direct == dir::CW) {
        settings->points.current_steps += steps;
    } else {
        settings->points.current_steps -= steps;
    }
}

bool extern_driver::saveCurrentPositionAsPoint(uint32_t point_number) {
    if(!validatePointNumber(point_number)) return false;
    if(!settings->points.is_calibrated) return false;

    settings->points.points[point_number] = settings->points.current_steps;
    if(point_number >= settings->points.count) {
        settings->points.count = point_number + 1;
    }

    return true;
}

uint32_t extern_driver::getCurrentSteps() {
    return settings->points.current_steps;
}

void extern_driver::resetCurrentPosition() {
    settings->points.current_steps = 0;
    settings->points.current_point = 0;
}

bool extern_driver::setCurrentPosition(uint32_t position) {
    if(!settings->points.is_calibrated) return false;
    settings->points.current_steps = position;
    return true;
}

bool extern_driver::gotoPoint(uint32_t point_number) {
    if(!validatePointNumber(point_number)) return false;
    if(!settings->points.is_calibrated) return false;
    if(point_number >= settings->points.count) return false;

    settings->points.target_point = point_number;

    // Определяем направление движения
    if(settings->points.points[point_number] > settings->points.current_steps) {
        settings->Direct = dir::CW;
    } else {
        settings->Direct = dir::CCW;
    }

    // Рассчитываем расстояние
    uint32_t target_distance = abs((int32_t)settings->points.points[point_number] -
                                 (int32_t)settings->points.current_steps);

    // Настраиваем таймеры/энкодер
    if(settings->mod_rotation == by_meter_timer_intermediate) {
        TimCountAllSteps->Instance->CNT = 0;
        TimCountAllSteps->Instance->ARR = target_distance;
    } else if(settings->mod_rotation == by_meter_enc_intermediate) {
        TimEncoder->Instance->CCR4 = settings->points.points[point_number];
        TimEncoder->Instance->CCR3 = TimEncoder->Instance->CCR4 +
            (settings->Direct == dir::CW ? settings->SlowdownDistance : -settings->SlowdownDistance);
    }

    return start();
}

void extern_driver::getPoints(points_response_t* points) {
    if(!points) return;
    memcpy(points->points, settings->points.points, sizeof(uint32_t) * settings->points.count);
    points->count = settings->points.count;
}

bool extern_driver::validatePosition(uint32_t position) {
    if(!settings->points.is_calibrated) return false;

    // Проверяем что позиция в допустимых пределах
    uint32_t max_pos = getMaxPosition();
    uint32_t min_pos = getMinPosition();

    return (position >= min_pos && position <= max_pos);
}

uint32_t extern_driver::getMaxPosition() const {
    // Максимальная позиция - это позиция второго концевика
    return CallSteps;
}

uint32_t extern_driver::getMinPosition() const {
    return 0;
}

void extern_driver::calculateTargetDistance(uint32_t position) {
    // Рассчитываем расстояние до целевой позиции
    uint32_t target_distance = abs((int32_t)position - (int32_t)settings->points.current_steps);

    // Инициализируем счетчики/компараторы
    TimCountAllSteps->Instance->CNT = 0;
    TimCountAllSteps->Instance->ARR = target_distance - settings->SlowdownDistance;
}

bool extern_driver::gotoPosition(uint32_t position) {
    if(!validatePosition(position)) {
        STM_LOG("Invalid position requested: %lu", position);
        return false;
    }

    // Проверяем состояние драйвера
    if(getDriverStatus() != DRIVER_OK) {
        STM_LOG("Driver error, cannot move");
        return false;
    }

    // Определяем направление движения
    if(position > settings->points.current_steps) {
        settings->Direct = dir::CW;
    } else if(position < settings->points.current_steps) {
        settings->Direct = dir::CCW;
    } else {
        // Уже в заданной позиции
        return true;
    }

    // Устанавливаем целевую позицию
    settings->Target = position;

    // Настраиваем компараторы в зависимости от режима
    switch(settings->mod_rotation) {
        case by_meter_timer:
        case by_meter_timer_intermediate:
            calculateTargetDistance(position);
            break;

        case by_meter_enc:
        case by_meter_enc_intermediate:
            TimEncoder->Instance->CCR4 = position;
            TimEncoder->Instance->CCR3 = TimEncoder->Instance->CCR4 +
                (settings->Direct == dir::CW ? settings->SlowdownDistance : -settings->SlowdownDistance);
            TimEncoder->Instance->CNT = settings->points.current_steps;
            break;

        default:
            STM_LOG("Unsupported rotation mode");
            return false;
    }

    return start();
}

void extern_driver::updateMotionCounter() {
    // Счетчики для работы с таймером и энкодером
    static uint32_t last_timer_count = 0;
    static uint32_t last_encoder_count = 0;

    // Обновление позиции в зависимости от режима работы
    switch(settings->mod_rotation) {
        case infinity:
            // В бесконечном режиме просто считаем шаги
            if(settings->motor == motor_t::stepper_motor) {
                uint32_t current_count = TimCountAllSteps->Instance->CNT;
                if(current_count != last_timer_count) {
                    updateCurrentSteps(current_count - last_timer_count);
                    last_timer_count = current_count;
                }
            }
            break;

        case infinity_enc:
            // Бесконечный режим с энкодером
            if(Status != statusMotor::STOPPED) {
                uint32_t current_enc = TimEncoder->Instance->CNT;
                int32_t delta = current_enc - last_encoder_count;
                // При больших скачках значений считаем что был переход через 0
                if(abs(delta) > 32768) {
                    if(settings->Direct == dir::CW) {
                        delta = (65535 - last_encoder_count) + current_enc;
                    } else {
                        delta = -((65535 - current_enc) + last_encoder_count);
                    }
                }
                updateCurrentSteps(delta);
                last_encoder_count = current_enc;
            }
            break;

        case by_meter_timer:
        case by_meter_timer_intermediate:
        case by_meter_timer_limit_switch:
            // Режимы работы по таймеру
            if(Status != statusMotor::STOPPED) {
                uint32_t current_count = TimCountAllSteps->Instance->CNT;
                if(current_count != last_timer_count) {
                    updateCurrentSteps(current_count - last_timer_count);
                    last_timer_count = current_count;

                    // Проверка необходимости торможения
                    if(Status == statusMotor::MOTION) {
                        uint32_t remaining = abs((int32_t)settings->Target - (int32_t)settings->points.current_steps);
                        if(remaining <= settings->SlowdownDistance) {
                            Status = statusMotor::BRAKING;
                        }
                    }
                }
            }
            break;

        case by_meter_enc:
        case by_meter_enc_intermediate:
            // Режимы работы с энкодером
            if(Status != statusMotor::STOPPED) {
                uint32_t current_enc = TimEncoder->Instance->CNT;
                int32_t delta = current_enc - last_encoder_count;

                // Обработка перехода через 0
                if(abs(delta) > 32768) {
                    if(settings->Direct == dir::CW) {
                        delta = (65535 - last_encoder_count) + current_enc;
                    } else {
                        delta = -((65535 - current_enc) + last_encoder_count);
                    }
                }

                updateCurrentSteps(delta);
                last_encoder_count = current_enc;

                // Проверка позиции для торможения
                if(Status == statusMotor::MOTION) {
                    if(settings->Direct == dir::CW) {
                        if(current_enc >= TimEncoder->Instance->CCR3 &&
                           current_enc <= TimEncoder->Instance->CCR4) {
                            Status = statusMotor::BRAKING;
                        }
                    } else {
                        if(current_enc <= TimEncoder->Instance->CCR3 &&
                           current_enc >= TimEncoder->Instance->CCR4) {
                            Status = statusMotor::BRAKING;
                        }
                    }
                }
            }
            break;
    }

    // Сброс счетчиков при остановке
    if(Status == statusMotor::STOPPED) {
        last_timer_count = 0;
        last_encoder_count = TimEncoder->Instance->CNT;
    }
}

bool extern_driver::isCalibrated() {
    return settings->points.is_calibrated;
}

//*******************************************************
void extern_driver::InitTim() {

}

double extern_driver::map(double x, double in_min, double in_max,
		double out_min, double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool extern_driver::waitForStop(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    while(Status != statusMotor::STOPPED) {
        if(HAL_GetTick() - start > timeout_ms) {
            STM_LOG("Stop timeout occurred");
            return false;
        }
        osDelay(1);
    }
    return true;
}

extern_driver::~extern_driver() {

}

extern_driver::extern_driver(settings_t *set, TIM_HandleTypeDef *timCount,
		TIM_HandleTypeDef *timFreq, uint32_t channelFreq,
		TIM_HandleTypeDef *timDebounce, TIM_HandleTypeDef *timENC) :
		settings(set), TimCountAllSteps(timCount), TimFrequencies(timFreq), ChannelClock(
				channelFreq), debounceTimer(timDebounce), TimEncoder(timENC) {
}

