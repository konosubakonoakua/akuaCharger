#include "stm32f4xx_hal.h"

// interfaces
uint32_t bq_write(uint8_t *pbuf, uint8_t size, uint8_t addr);
uint32_t bq_read(uint8_t *pbuf, uint8_t size, uint8_t addr);
void bq_turn_on();
extern void die(uint16_t ms);
extern void led_off();
extern void led_on();