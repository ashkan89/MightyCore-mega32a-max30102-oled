/* gfx.h -- framebuffer drawing primitives (no stdio, no malloc) */
#ifndef GFX_H
#define GFX_H
#include "config.h"
#include <avr/pgmspace.h>

#define C_BLACK  0
#define C_WHITE  1
#define C_XOR    2

void gfx_clear(void);
void gfx_pixel(int16_t x, int16_t y, uint8_t c);
void gfx_hline(int16_t x, int16_t y, int16_t w, uint8_t c);
void gfx_vline(int16_t x, int16_t y, int16_t h, uint8_t c);
void gfx_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t c);
void gfx_fill(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t c);
void gfx_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t c);
void gfx_rrect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t c);
void gfx_dither(int16_t x, int16_t y, int16_t w, int16_t h);

uint8_t gfx_char(int16_t x, int16_t y, char ch, uint8_t sc, uint8_t c);
uint8_t gfx_text(int16_t x, int16_t y, const char *s, uint8_t sc, uint8_t c);
uint8_t gfx_text_P(int16_t x, int16_t y, PGM_P s, uint8_t sc, uint8_t c);
uint8_t gfx_text_w(const char *s, uint8_t sc);
uint8_t gfx_text_w_P(PGM_P s, uint8_t sc);

uint8_t gfx_num(int16_t x, int16_t y, int32_t v, uint8_t sc, uint8_t c);
uint8_t gfx_num_pad(int16_t x, int16_t y, uint32_t v, uint8_t digits, uint8_t sc, uint8_t c);
uint8_t gfx_num_dec(int16_t x, int16_t y, int32_t v_x10, uint8_t sc, uint8_t c);
uint8_t gfx_num_w(int32_t v, uint8_t sc);

/* 7-segment style big numerals, scalable */
void gfx_seg_digit(int16_t x, int16_t y, uint8_t w, uint8_t h, uint8_t t,
                   int8_t digit, uint8_t c);
void gfx_seg_num(int16_t x, int16_t y, uint8_t w, uint8_t h, uint8_t t,
                 uint8_t gap, uint16_t v, uint8_t digits, uint8_t blank_lead,
                 uint8_t c);

void gfx_bitmap_P(int16_t x, int16_t y, const uint8_t *bmp, uint8_t w, uint8_t h, uint8_t c);
void gfx_heart(int16_t x, int16_t y, uint8_t big, uint8_t c);
void gfx_progress(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t pct);

#endif
