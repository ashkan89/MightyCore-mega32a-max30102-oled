#include "dbg.h"

#if DBG_UART

#include "sys.h"
#include "ppg.h"
#include "max30102.h"
#include "i2c.h"
#include <avr/pgmspace.h>

/* U2X doubles the sampler rate, which halves the divisor error.  At 16 MHz
 * and 38400 that lands within 0.2 %, against 2.1 % for 115200 -- worth the
 * lower rate for a link that has to be trustworthy when nothing else is. */
#define UBRR_U2X ((uint16_t)((F_CPU / (8UL * DBG_BAUD)) - 1UL))

static uint32_t s_loops;

void dbg_loop(void) { s_loops++; }

static void tx(char c)
{
    uint16_t guard = 0;
    while (!(UCSRA & (1 << UDRE)))
        if (++guard > 20000) return;      /* never hang the main loop on it */
    UDR = (uint8_t)c;
}

static void tx_P(const char *s)
{
    char c;
    while ((c = (char)pgm_read_byte(s++)) != '\0') tx(c);
}

static void tx_u32(uint32_t v)
{
    char b[10];
    uint8_t n = 0;
    if (!v) { tx('0'); return; }
    while (v) { b[n++] = (char)('0' + (v % 10)); v /= 10; }
    while (n) tx(b[--n]);
}

/* "name=value" pairs, so the line stays readable as fields come and go */
static void f_hex(const char *name_P, uint8_t v)
{
    static const char h[] PROGMEM = "0123456789ABCDEF";
    tx(' '); tx_P(name_P); tx('=');
    tx((char)pgm_read_byte(&h[v >> 4]));
    tx((char)pgm_read_byte(&h[v & 0x0F]));
}

static void f_dec(const char *name_P, uint32_t v)
{
    tx(' '); tx_P(name_P); tx('='); tx_u32(v);
}

void dbg_init(void)
{
    UBRRH = (uint8_t)(UBRR_U2X >> 8);
    UBRRL = (uint8_t)(UBRR_U2X & 0xFF);
    UCSRA = (1 << U2X);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);   /* 8N1 */
    UCSRB = (1 << TXEN);
    tx_P(PSTR("\r\n" FW_NAME " " FW_VERSION " diag\r\n"));
}

/* One line per crossing: the interval, the pulse amplitude that produced
 * it, and the reason code.  This is what distinguishes the three failure
 * shapes that all look like "jumping numbers" on the display -- firing
 * twice per beat (alternating short/long), missing beats (intervals at
 * multiples of the true one), and noise (no pattern at all). */
void dbg_beat(uint16_t ibi_ms, uint16_t amp, uint8_t code)
{
    static const char why[] PROGMEM = "OSLACW";  /* Ok Short Long Acq Cont Weak */
    tx_P(PSTR("B ibi="));
    tx_u32(ibi_ms);
    tx_P(PSTR(" amp="));
    tx_u32(amp);
    tx_P(PSTR(" r="));
    tx((char)pgm_read_byte(&why[(code < 6) ? code : 5]));
    tx_P(PSTR("\r\n"));
}

void dbg_service(void)
{
    static uint32_t next_ms, loop_mark;
    static uint8_t  started;
    uint32_t now = millis();
    uint8_t  mode, spo2, fifo, lines;

    if (started && (int32_t)(now - next_ms) < 0) return;
    started = 1;
    next_ms = now + 2000;

    max30102_readback(&mode, &spo2, &fifo);
    lines = i2c_lines();

    /* --- identity and configuration: these should never change --- */
    f_dec(PSTR("t"),    now / 1000UL);
    f_dec(PSTR("up"),   max30102_present());
    f_hex(PSTR("id"),   max30102_part_id());     /* 15                     */
    f_hex(PSTR("mode"), mode);                   /* 03 = RED+IR            */
    f_hex(PSTR("spo2"), spo2);                   /* 2F = 4096nA/400Hz/411us*/
    f_hex(PSTR("fifo"), fifo);                   /* rollover on, avg 4     */

    /* --- stream health --- */
    f_dec(PSTR("sps"),  ppg.sps);                /* samples/s, want ~100   */
    f_dec(PSTR("fs"),   ppg.fs_x100 / 100UL);
    f_dec(PSTR("lps"),  (s_loops - loop_mark) / 2); /* main-loop passes/s  */
    loop_mark = s_loops;

    /* --- detection --- */
    f_dec(PSTR("ir"),   ppg.dc_ir);
    f_dec(PSTR("refl"), ppg.refl_ir);      /* what the detector compares  */
    f_dec(PSTR("red"),  ppg.dc_red);
    f_dec(PSTR("base"), ppg.base_ir);            /* learned idle level     */
    f_dec(PSTR("th"),   ppg.finger_th);          /* ir has to pass this    */
    f_dec(PSTR("fgr"),  ppg.finger);
    f_hex(PSTR("led"),  ppg.led_ir);

    /* --- measurement --- */
    f_dec(PSTR("bpm"),  ppg.bpm_x10 / 10U);
    f_dec(PSTR("sp"),   ppg.spo2_x10 / 10U);
    f_dec(PSTR("bts"),  ppg.beats);
    f_dec(PSTR("rej"),  ppg.rejects);
    f_dec(PSTR("val"),  ppg.valid);

    /* --- bus health: err, stk and a non-3 ln all mean trouble --- */
    f_dec(PSTR("err"),  max30102_errors());
    f_dec(PSTR("stk"),  i2c_stuck_count());
    f_hex(PSTR("tw"),   i2c_last_status());
    f_dec(PSTR("stg"),  i2c_last_stage());
    f_dec(PSTR("ln"),   (uint32_t)(lines & 3));  /* 3 = both idle high     */
    tx_P(PSTR("\r\n"));
}

#endif /* DBG_UART */
