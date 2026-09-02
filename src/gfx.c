#include "gfx.h"
#include "ssd1306.h"
#include "font5x7.h"

void gfx_clear(void)
{
    uint16_t i;
    for (i = 0; i < sizeof(fb); i++) fb[i] = 0;
}

void gfx_pixel(int16_t x, int16_t y, uint8_t c)
{
    uint16_t idx;
    uint8_t  m;
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;
    idx = (uint16_t)((y >> 3) * OLED_W + x);
    m   = (uint8_t)(1 << (y & 7));
    if      (c == C_WHITE) fb[idx] |= m;
    else if (c == C_BLACK) fb[idx] = (uint8_t)(fb[idx] & (uint8_t)~m);
    else                   fb[idx] ^= m;
}

void gfx_hline(int16_t x, int16_t y, int16_t w, uint8_t c)
{
    if (y < 0 || y >= OLED_H) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > OLED_W) w = OLED_W - x;
    while (w-- > 0) gfx_pixel(x++, y, c);
}

void gfx_vline(int16_t x, int16_t y, int16_t h, uint8_t c)
{
    if (x < 0 || x >= OLED_W) return;
    if (y < 0) { h += y; y = 0; }
    if (y + h > OLED_H) h = OLED_H - y;
    while (h-- > 0) gfx_pixel(x, y++, c);
}

void gfx_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t c)
{
    if (w <= 0 || h <= 0) return;
    gfx_hline(x, y, w, c);
    gfx_hline(x, y + h - 1, w, c);
    gfx_vline(x, y, h, c);
    gfx_vline(x + w - 1, y, h, c);
}

void gfx_fill(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t c)
{
    int16_t j;
    for (j = 0; j < h; j++) gfx_hline(x, y + j, w, c);
}

/* rounded rectangle: plain rect with the four corner pixels knocked out */
void gfx_rrect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t c)
{
    if (w < 3 || h < 3) { gfx_rect(x, y, w, h, c); return; }
    gfx_hline(x + 1, y, w - 2, c);
    gfx_hline(x + 1, y + h - 1, w - 2, c);
    gfx_vline(x, y + 1, h - 2, c);
    gfx_vline(x + w - 1, y + 1, h - 2, c);
}

/* 50 percent checkerboard, used for secondary / disabled shading */
void gfx_dither(int16_t x, int16_t y, int16_t w, int16_t h)
{
    int16_t i, j;
    for (j = 0; j < h; j++)
        for (i = ((j & 1) ? 1 : 0); i < w; i += 2)
            gfx_pixel(x + i, y + j, C_WHITE);
}

void gfx_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t c)
{
    int16_t dx = (int16_t)(x1 > x0 ? x1 - x0 : x0 - x1);
    int16_t dy = (int16_t)(y1 > y0 ? y1 - y0 : y0 - y1);
    int16_t sx = (int16_t)(x0 < x1 ? 1 : -1);
    int16_t sy = (int16_t)(y0 < y1 ? 1 : -1);
    int16_t err = (int16_t)(dx - dy), e2;
    for (;;) {
        gfx_pixel(x0, y0, c);
        if (x0 == x1 && y0 == y1) break;
        e2 = (int16_t)(err << 1);
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/* ------------------------------ text ------------------------------ */
uint8_t gfx_char(int16_t x, int16_t y, char ch, uint8_t sc, uint8_t c)
{
    uint8_t col, row, bits;
    uint8_t u = (uint8_t)ch;
    if (u < 0x20 || u > 0x7F) u = '?';
    for (col = 0; col < 5; col++) {
        bits = pgm_read_byte(&font5x7[u - 0x20][col]);
        for (row = 0; row < 7; row++) {
            if (bits & (1 << row)) {
                if (sc == 1) gfx_pixel(x + col, y + row, c);
                else gfx_fill(x + (int16_t)col * sc, y + (int16_t)row * sc, sc, sc, c);
            }
        }
    }
    return (uint8_t)(6 * sc);
}

uint8_t gfx_text(int16_t x, int16_t y, const char *s, uint8_t sc, uint8_t c)
{
    int16_t x0 = x;
    while (*s) x += gfx_char(x, y, *s++, sc, c);
    return (uint8_t)(x - x0);
}

uint8_t gfx_text_P(int16_t x, int16_t y, PGM_P s, uint8_t sc, uint8_t c)
{
    int16_t x0 = x;
    char ch;
    while ((ch = (char)pgm_read_byte(s++)) != 0) x += gfx_char(x, y, ch, sc, c);
    return (uint8_t)(x - x0);
}

uint8_t gfx_text_w(const char *s, uint8_t sc)
{
    uint8_t n = 0;
    while (*s++) n++;
    return (uint8_t)(n * 6 * sc);
}

uint8_t gfx_text_w_P(PGM_P s, uint8_t sc)
{
    uint8_t n = 0;
    while (pgm_read_byte(s++)) n++;
    return (uint8_t)(n * 6 * sc);
}

/* ----------------------------- numbers ---------------------------- */
static uint8_t utoa_rev(uint32_t v, char *buf)   /* returns length */
{
    uint8_t n = 0;
    do { buf[n++] = (char)('0' + (v % 10)); v /= 10; } while (v && n < 10);
    return n;
}

uint8_t gfx_num_w(int32_t v, uint8_t sc)
{
    char b[12];
    uint8_t n = utoa_rev((uint32_t)(v < 0 ? -v : v), b);
    if (v < 0) n++;
    return (uint8_t)(n * 6 * sc);
}

uint8_t gfx_num(int16_t x, int16_t y, int32_t v, uint8_t sc, uint8_t c)
{
    char b[12];
    uint8_t n, i;
    int16_t x0 = x;
    if (v < 0) { x += gfx_char(x, y, '-', sc, c); v = -v; }
    n = utoa_rev((uint32_t)v, b);
    for (i = n; i > 0; i--) x += gfx_char(x, y, b[i - 1], sc, c);
    return (uint8_t)(x - x0);
}

uint8_t gfx_num_pad(int16_t x, int16_t y, uint32_t v, uint8_t digits, uint8_t sc, uint8_t c)
{
    char b[12];
    uint8_t n, i;
    int16_t x0 = x;
    n = utoa_rev(v, b);
    for (i = n; i < digits; i++) x += gfx_char(x, y, '0', sc, c);
    for (i = n; i > 0; i--)      x += gfx_char(x, y, b[i - 1], sc, c);
    return (uint8_t)(x - x0);
}

uint8_t gfx_num_dec(int16_t x, int16_t y, int32_t v_x10, uint8_t sc, uint8_t c)
{
    int16_t x0 = x;
    if (v_x10 < 0) { x += gfx_char(x, y, '-', sc, c); v_x10 = -v_x10; }
    x += gfx_num(x, y, v_x10 / 10, sc, c);
    x += gfx_char(x, y, '.', sc, c);
    x += gfx_char(x, y, (char)('0' + (v_x10 % 10)), sc, c);
    return (uint8_t)(x - x0);
}

/* -------------------- 7-segment scalable numerals ------------------ */
/* bit0..bit6 = A B C D E F G */
static const uint8_t seg_map[10] PROGMEM = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

void gfx_seg_digit(int16_t x, int16_t y, uint8_t w, uint8_t h, uint8_t t,
                   int8_t digit, uint8_t c)
{
    uint8_t m, half, vlen;
    if (digit < 0 || digit > 9) return;
    m    = pgm_read_byte(&seg_map[digit]);
    half = (uint8_t)((h - t) / 2);
    vlen = (uint8_t)(half - t + 1);
    if (m & 0x01) gfx_fill(x + t,     y,            w - 2 * t, t,    c); /* A */
    if (m & 0x02) gfx_fill(x + w - t, y + t,        t,         vlen, c); /* B */
    if (m & 0x04) gfx_fill(x + w - t, y + half + t, t,         vlen, c); /* C */
    if (m & 0x08) gfx_fill(x + t,     y + h - t,    w - 2 * t, t,    c); /* D */
    if (m & 0x10) gfx_fill(x,         y + half + t, t,         vlen, c); /* E */
    if (m & 0x20) gfx_fill(x,         y + t,        t,         vlen, c); /* F */
    if (m & 0x40) gfx_fill(x + t,     y + half,     w - 2 * t, t,    c); /* G */
}

void gfx_seg_num(int16_t x, int16_t y, uint8_t w, uint8_t h, uint8_t t,
                 uint8_t gap, uint16_t v, uint8_t digits, uint8_t blank_lead,
                 uint8_t c)
{
    int8_t  d[5];
    uint8_t i;
    uint8_t lead = 1;
    if (digits > 5) digits = 5;
    for (i = digits; i > 0; i--) { d[i - 1] = (int8_t)(v % 10); v /= 10; }
    for (i = 0; i < digits; i++) {
        if (blank_lead && lead && d[i] == 0 && i < (uint8_t)(digits - 1)) {
            /* suppressed leading zero */
        } else {
            lead = 0;
            gfx_seg_digit(x, y, w, h, t, d[i], c);
        }
        x += (int16_t)(w + gap);
    }
}

/* ------------------------------ icons ------------------------------ */
static const uint8_t heart8[8] PROGMEM = {  /* 8x8, row-major, MSB left */
    0x66, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C, 0x18, 0x00
};
/* half-width mask of a 13x12 heart; bit i = pixel i columns from centre */
static const uint8_t heart12[12] PROGMEM = {
    0x1C, 0x3E, 0x7F, 0x7F, 0x7F, 0x7F, 0x3F, 0x1F, 0x0F, 0x07, 0x03, 0x01
};

void gfx_bitmap_P(int16_t x, int16_t y, const uint8_t *bmp, uint8_t w, uint8_t h, uint8_t c)
{
    uint8_t r, i;
    for (r = 0; r < h; r++) {
        uint8_t b = pgm_read_byte(&bmp[r]);
        for (i = 0; i < w && i < 8; i++)
            if (b & (uint8_t)(0x80 >> i)) gfx_pixel(x + i, y + r, c);
    }
}

void gfx_heart(int16_t x, int16_t y, uint8_t big, uint8_t c)
{
    uint8_t r, i;
    if (!big) { gfx_bitmap_P(x, y, heart8, 8, 8, c); return; }
    for (r = 0; r < 12; r++) {
        uint8_t b = pgm_read_byte(&heart12[r]);
        for (i = 0; i < 7; i++)
            if (b & (uint8_t)(1 << i)) {
                gfx_pixel(x + 6 - i, y + r, c);
                gfx_pixel(x + 6 + i, y + r, c);
            }
    }
}

void gfx_progress(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t pct)
{
    int16_t f;
    if (pct > 100) pct = 100;
    gfx_rrect(x, y, w, h, C_WHITE);
    f = (int16_t)(((int32_t)(w - 4) * pct) / 100);
    if (f > 0) gfx_fill(x + 2, y + 2, f, h - 4, C_WHITE);
}
