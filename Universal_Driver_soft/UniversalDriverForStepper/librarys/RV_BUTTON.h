#ifndef RV_BUTTON_h
#define RV_BUTTON_h

#include <stdint.h>
#include "stm32f3xx_hal.h"

#pragma pack(push, 1)
typedef struct {
	bool btn_state: 1;
	bool btn_flag: 1;
	bool hold_flag: 1;
	bool counter_flag: 1;
	bool isHolded_f: 1;
	bool isRelease_f: 1;
	bool isPress_f: 1;
	bool step_flag: 1;
	bool oneClick_f: 1;
	bool isOne_f: 1;
} buttonFlags;
#pragma pack(pop)

class RV_BUTTON {
  public:
		RV_BUTTON(GPIO_TypeDef *port, uint16_t pin, bool type, bool dir);//создание экземпляра класса кнопки, порт, вывод, type (HIGH_PULL / LOW_PULL) и направление dir (NORM_OPEN / NORM_CLOSE)
		//HIGH_PULL - подтяжка к VCC
		//LOW_PULL - подтяжка к GND
		//NORM_OPEN - нормально-разомкнутая
		//NORM_CLOSE - нормально-замкнутая
		void setDebounce(uint16_t debounce);//установка таймаута антидребезга (по умолчанию 120 мс)
		void setTimeout(uint16_t timeout);//установка таймаута удержания (по умолчанию 500 мс)
		void setStepTimeout(uint16_t step_timeout);//установка таймаута между инкрементами (по умолчанию 400 мс)
		void setType(bool type);//установка типа кнопки (HIGH_PULL / LOW_PULL)
		void setDirection(bool dir);//установка направления (разомкнута/замкнута по умолчанию - NORM_OPEN, NORM_CLOSE)
		void tick();//опрос кнопки
		void tick(bool state);//опрос внешнего значения (0 нажато, 1 не нажато)
		bool isPress();//возвращает true при нажатии на кнопку. Сбрасывается после вызова
		bool isRelease();//возвращает true при отпускании кнопки. Сбрасывается после вызова
		bool isClick();//возвращает true при клике. Сбрасывается после вызова
		bool isHolded();//возвращает true при удержании дольше timeout. Сбрасывается после вызова
		bool isHold();//возвращает true при нажатой кнопке, не сбрасывается
		bool state();//возвращает состояние кнопки

		bool isSingle();//возвращает true при одиночном клике. Сбрасывается после вызова
		bool isDouble();//возвращает true при двойном клике. Сбрасывается после вызова
		bool isTriple();//возвращает true при тройном клике. Сбрасывается после вызова
	
		bool hasClicks();//проверка на наличие кликов. Сбрасывается после вызова
		uint8_t getClicks();//возвращает количество кликов
	
		bool isStep();//возвращает true по таймеру setStepTimeout
	
  private:
		void init();
    buttonFlags flags;
    GPIO_TypeDef *_PORT;
		uint16_t _PIN;
		uint16_t _debounce;
		uint16_t _timeout;
		uint16_t _step_timeout;
		uint8_t btn_counter, last_counter;	
		uint32_t btn_timer;		
		bool _inv_state;
		bool _mode;
		bool _type;
};

#define HIGH_PULL 0
#define LOW_PULL 1
#define NORM_OPEN 0
#define NORM_CLOSE 1
#define MANUAL 0
#define AUTO 1
 
#endif
