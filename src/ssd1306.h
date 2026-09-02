/* ssd1306.h -- 128x64 SSD1306 over hardware SPI, full RAM framebuffer */
#ifndef SSD1306_H
#define SSD1306_H
#include "config.h"

extern uint8_t fb[OLED_W * OLED_H / 8];   /* page-major framebuffer */

void spi_init(void);
void ssd1306_init(void);
void ssd1306_flush(void);                 /* push framebuffer to panel */
void ssd1306_contrast(uint8_t c);
void ssd1306_flip(uint8_t on);
void ssd1306_power(uint8_t on);
void ssd1306_charge_pump(uint8_t on);

#endif
