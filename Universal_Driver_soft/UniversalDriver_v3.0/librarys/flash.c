#include "flash.h"


void FLASH_WriteSettings(settings_t params, uint32_t pageAdr) {

	uint8_t i;

	// Опеределим, сколько блоков памяти у нас определено под параметры
	// Это необходимо для определения количества циклов чтения/записи параметров
	uint32_t PARAMS_WORD_CNT =	sizeof(settings_t) / sizeof(uint32_t); // Расчитывается исходя из размера структуры в памяти МК деленного на размер блока (4 байта)

	uint32_t *source_adr = (void *)&params;

	flash_unlock();                                                                 // Разблокируем память для записи
	flash_erase_page(pageAdr);                                                       // Очистим страницу памяти параметров №0

	for (i = 0; i < PARAMS_WORD_CNT; ++i) {
                flash_write((uint32_t)(pageAdr + i*4), *(source_adr + i));        // Запишем новое значение памяти
        }

	flash_lock();                                                                   // Заблокируем Flash

}
void Flash_ReadParams(settings_t *params, uint32_t adr) {

	uint32_t *dest_adr = (void *)params; 
	uint32_t *source_adr = (uint32_t *)adr;
	// Определяем адрес, куда будем писать
	// Опеределим, сколько блоков памяти у нас определено под параметры
	// Это необходимо для определения количества циклов чтения/записи параметров
	uint32_t PARAMS_WORD_CNT =	sizeof(settings_t) / sizeof(uint32_t); // Расчитывается исходя из размера структуры в памяти МК деленного на размер блока (4 байта)

	for (uint16_t i=0; i < PARAMS_WORD_CNT; ++i) {                                  // В цикле производим чтение
		*(dest_adr + i) = *(__IO uint32_t*)(source_adr + i);                    // Само чтение
	}
}

uint8_t flash_ready(void) {
	return !(FLASH->SR & FLASH_SR_BSY);
}

//void flash_erase_all_pages(void) {
//    FLASH->CR |= FLASH_CR_MER;
//    FLASH->CR |= FLASH_CR_STRT;
//    while(!flash_ready())
//    	;
//    FLASH->CR &= FLASH_CR_MER;
//}

void flash_erase_page(uint32_t address) {
    FLASH->CR|= FLASH_CR_PER;
    FLASH->AR = address;
    FLASH->CR|= FLASH_CR_STRT;
    while(!flash_ready())
    	;
    FLASH->CR&= ~FLASH_CR_PER;
}


void flash_unlock(void) {
	  FLASH->KEYR = FLASH_KEY1;
	  FLASH->KEYR = FLASH_KEY2;
}

void flash_lock(void) {
	FLASH->CR |= FLASH_CR_LOCK;
}


void flash_write(uint32_t address,uint32_t data) {

	FLASH->CR |= FLASH_CR_PG;
	while(!flash_ready())
		;
    *(__IO uint16_t*)address = (uint16_t)data;
	while(!flash_ready())
		;
	address+=2;
	data>>=16;
    *(__IO uint16_t*)address = (uint16_t)data;
	while(!flash_ready())
		;
    FLASH->CR &= ~(FLASH_CR_PG);
}

