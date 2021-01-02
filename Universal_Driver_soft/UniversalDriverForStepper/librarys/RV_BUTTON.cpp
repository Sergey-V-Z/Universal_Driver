#include "RV_BUTTON.h"

//создание экземпляра класса кнопки, порт, вывод, type (HIGH_PULL / LOW_PULL) и направление dir (NORM_OPEN / NORM_CLOSE)
RV_BUTTON::RV_BUTTON(GPIO_TypeDef *port, uint16_t pin, bool type, bool dir) {
	_PORT = port;
	_PIN = pin;
	RV_BUTTON::init();
	RV_BUTTON::setType(type);
	_inv_state = dir;
}
//инициализация
void RV_BUTTON::init() {
	_mode = false;
	btn_timer = 0;
	btn_counter = 0;
	last_counter = 0;
	_debounce = 120;
	_timeout = 500;
	_step_timeout = 400;
	_inv_state = NORM_OPEN;
	RV_BUTTON::setType(HIGH_PULL);
}
//установка таймаута антидребезга (по умолчанию 120 мс)
void RV_BUTTON::setDebounce(uint16_t debounce) {
	_debounce = debounce;
}
//установка таймаута удержания (по умолчанию 500 мс)
void RV_BUTTON::setTimeout(uint16_t timeout) {
	_timeout = timeout;
}
//установка таймаута между инкрементами (по умолчанию 400 мс)
void RV_BUTTON::setStepTimeout(uint16_t step_timeout) {
	_step_timeout = step_timeout;
}
//установка типа кнопки (HIGH_PULL / LOW_PULL)
void RV_BUTTON::setType(bool type) {
	_type = type;
}
//установка направления (разомкнута/замкнута по умолчанию - NORM_OPEN, NORM_CLOSE)
void RV_BUTTON::setDirection(bool dir) {
	_inv_state = dir;
}
//возвращает true при нажатии на кнопку. Сбрасывается после вызова
bool RV_BUTTON::isPress() {
	if (flags.isPress_f) {
		flags.isPress_f = false;
		return true;
	}
	else
		return false;
}
//возвращает true при отпускании кнопки. Сбрасывается после вызова
bool RV_BUTTON::isRelease() {
	if (flags.isRelease_f) {
		flags.isRelease_f = false;
		return true;
	}
	else
		return false;
}
//возвращает true при клике. Сбрасывается после вызова
bool RV_BUTTON::isClick() {	
	if (flags.isOne_f) {
		flags.isOne_f = false;
		return true;
	}
	else
		return false;
}
//возвращает true при удержании дольше timeout. Сбрасывается после вызова
bool RV_BUTTON::isHolded() {
	if (flags.isHolded_f) {
		flags.isHolded_f = false;
		return true;
	}
	else
		return false;
}
//возвращает true при нажатой кнопке, не сбрасывается
bool RV_BUTTON::isHold() {
	if (flags.step_flag)
		return true;
	else
		return false;
}
//возвращает состояние кнопки
bool RV_BUTTON::state() {
	return flags.btn_state;
}
//возвращает true при одиночном клике. Сбрасывается после вызова
bool RV_BUTTON::isSingle() {
	if (flags.counter_flag && last_counter == 1) {
		flags.counter_flag = false;
		return true;
	}
	else
		return false;
}
//возвращает true при двойном клике. Сбрасывается после вызова
bool RV_BUTTON::isDouble() {
	if (flags.counter_flag && last_counter == 2) {
		flags.counter_flag = false;
		return true;
	}
	else
		return false;
}
//возвращает true при тройном клике. Сбрасывается после вызова
bool RV_BUTTON::isTriple() {
	if (flags.counter_flag && last_counter == 3) {
		flags.counter_flag = false;
		return true;
	}
	else
		return false;
}
//проверка на наличие кликов. Сбрасывается после вызова
bool RV_BUTTON::hasClicks() {
	if (flags.counter_flag) {
		flags.counter_flag = false;
		return true;
	}
	else
		return false;
}
//возвращает количество кликов
uint8_t RV_BUTTON::getClicks() {
	return last_counter;	
}
//возвращает true по таймеру setStepTimeout
bool RV_BUTTON::isStep() {
	if (flags.step_flag && (HAL_GetTick() - btn_timer >= _step_timeout)) {
		btn_timer = HAL_GetTick();
		return true;
	}
	else
		return false;
}
//опрос внешнего значения (0 нажато, 1 не нажато)
void RV_BUTTON::tick(bool state) {
	_mode = true;
	flags.btn_state = state ^ _inv_state;
	_mode = false;
}
//опрос кнопки
void RV_BUTTON::tick() {	
	if (!_mode) flags.btn_state = !HAL_GPIO_ReadPin(_PORT, _PIN) ^ (_inv_state ^ _type);
	
  if (flags.btn_state && !flags.btn_flag && (HAL_GetTick() - btn_timer >= _debounce)) {
    flags.btn_flag = true;
    btn_counter++;
    btn_timer = HAL_GetTick();
    flags.isPress_f = true;
		flags.oneClick_f = true;
  }
  if (!flags.btn_state && flags.btn_flag) {
    flags.btn_flag = false;
    flags.hold_flag = false;
    flags.isRelease_f = true;
		btn_timer = HAL_GetTick();
		flags.step_flag = false;
		if (flags.oneClick_f) {
			flags.oneClick_f = false;
			flags.isOne_f = true;
		}
  }
  if (flags.btn_flag && flags.btn_state && (HAL_GetTick() - btn_timer >= _timeout) && !flags.hold_flag) {
    flags.hold_flag = true;
    btn_counter = 0;
    flags.isHolded_f = true;
		flags.step_flag = true;
		flags.oneClick_f = false;
		btn_timer = HAL_GetTick();
  }
  
  if ((HAL_GetTick() - btn_timer >= _timeout) && (btn_counter != 0)) {    
    last_counter = btn_counter;
    btn_counter = 0;
		flags.counter_flag = true;
  }
}
