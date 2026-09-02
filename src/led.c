#include "led.h"
#include "sys.h"
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define LED_PORT  PORTB
#define LED_DDR   DDRB
#define LED_BIT   PB0        /* MightyCore LED_BUILTIN, standard pinout */

/* ---- brightness levels on the 0..255 scale ---- */
#define D_BEAT       8       /* locked keep-alive: barely lit  */
#define D_SEARCH    90       /* acquisition blip               */
#define D_COUNT    200       /* pre-sleep blink                */
#define D_FLASH    255       /* heartbeat                      */

/* ---- pattern periods, in phase units per millisecond ----
 * The phase is a 16-bit accumulator; the top byte indexes the pattern, so
 * one full cycle is 65536 units and the rate is 65536 / period_ms. */
#define RATE_MS(ms)  ((uint16_t)(65536UL / (ms)))
#define BREATHE_RATE RATE_MS(1500)    /* calm 1.5 s breath */
#define SEARCH_RATE  RATE_MS(900)
#define COUNT_RATE   RATE_MS(500)     /* 2 Hz */

static volatile uint8_t s_duty;      /* 0..255, read by the ISR */
static uint8_t  s_mode;
static uint16_t s_phase;             /* pattern phase, top byte = position */
static uint32_t s_phase_ms;          /* when the phase was last advanced   */
static uint32_t s_flash_until;

/* Quarter sine, sin(i * 90/16 deg) * 255.  Interpolated to 8 bits below, so
 * 17 bytes of flash buy a curve with no visible quantisation. */
static const uint8_t qsin[17] PROGMEM = {
      0,  25,  50,  74,  98, 120, 142, 162, 180,
    197, 212, 225, 236, 244, 250, 254, 255
};

/* sin(pi * p / 256), 0..255: rises and falls once across the full phase. */
static uint8_t half_sine(uint8_t p)
{
    uint8_t q = (uint8_t)((p < 128) ? p : (255 - p));   /* fold to 0..127 */
    uint8_t i = (uint8_t)(q >> 3);                      /* table index    */
    uint8_t f = (uint8_t)(q & 7);                       /* interpolation  */
    uint8_t a = pgm_read_byte(&qsin[i]);
    uint8_t b = pgm_read_byte(&qsin[i + 1]);
    return (uint8_t)(a + (uint8_t)(((uint16_t)(b - a) * f) >> 3));
}

/* Perceived brightness goes roughly as the square root of the duty cycle, so
 * squaring a linear ramp is what makes the fade look even.  The old table
 * claimed to do this but had only 32 output levels to spend, which is why the
 * bottom of the fade was five identical zeros and the top jumped in fours. */
static uint8_t breathe_duty(uint8_t p)
{
    uint16_t lin = half_sine(p);
    return (uint8_t)((lin * lin) >> 8);
}

void led_init(void)
{
    LED_DDR  |= (uint8_t)(1 << LED_BIT);
    LED_PORT &= (uint8_t)~(1 << LED_BIT);
    s_duty     = 0;
    s_phase    = 0;
    s_phase_ms = millis();

    /* CTC, clk/64 -> 250 kHz.  OCR2 is reloaded by the ISR with each slice. */
    TCCR2 = (uint8_t)((1 << WGM21) | (1 << CS22));
    TCNT2 = 0;
    OCR2  = 255;
    TIMSK = (uint8_t)((TIMSK & (uint8_t)~(1 << TOIE2)) | (1 << OCIE2));
}

void led_stop(void)
{
    TIMSK &= (uint8_t)~(1 << OCIE2);
    TCCR2 = 0;
    s_duty = 0;
    LED_PORT &= (uint8_t)~(1 << LED_BIT);
}

void led_duty(uint8_t d) { s_duty = d; }

void led_mode(uint8_t m)
{
    if (m == s_mode) return;
    s_mode     = m;
    s_phase    = 0;
    s_phase_ms = millis();
    if (m == LED_OFF) led_duty(0);
}

void led_flash(void) { s_flash_until = millis() + 70; }

void led_service(void)
{
    uint32_t now = millis();
    uint16_t el  = (uint16_t)(now - s_phase_ms);

    /* The phase is advanced by elapsed time rather than one table step per
     * call.  The main loop is not isochronous -- a redraw pass costs far more
     * than a FIFO pass -- and stepping per call made the fade rate wobble
     * with it.  Deriving the level from the clock also means a heartbeat
     * flash no longer freezes the pattern underneath it. */
    if (el > 200) el = 200;             /* after a stall, resume, don't jump */
    s_phase_ms = now;

    /* a heartbeat flash overrides whatever the pattern is doing */
    if (s_flash_until && (int32_t)(now - s_flash_until) < 0) {
        led_duty(D_FLASH);
        return;
    }
    s_flash_until = 0;

    switch (s_mode) {
        case LED_BREATHE:
            s_phase = (uint16_t)(s_phase + (uint16_t)((uint32_t)el * BREATHE_RATE));
            led_duty(breathe_duty((uint8_t)(s_phase >> 8)));
            break;

        case LED_SEARCH: {                             /* quick double blip */
            uint8_t p;
            s_phase = (uint16_t)(s_phase + (uint16_t)((uint32_t)el * SEARCH_RATE));
            p = (uint8_t)(s_phase >> 8);
            led_duty((uint8_t)((p < 24 || (p >= 48 && p < 72)) ? D_SEARCH : 0));
            break;
        }

        case LED_BEAT:
            led_duty(D_BEAT);                          /* dim keep-alive */
            break;

        case LED_COUNTDOWN:
            s_phase = (uint16_t)(s_phase + (uint16_t)((uint32_t)el * COUNT_RATE));
            led_duty((uint8_t)((s_phase & 0x8000) ? D_COUNT : 0));
            break;

        default:
            led_duty(0);
            break;
    }
}

/* Two slices per PWM period: the ON slice lasts s_duty ticks and the OFF
 * slice the remaining 256 - s_duty, so the period is constant at 256 ticks
 * whatever the duty.  OCR2 is never loaded with 0 -- in CTC the counter has
 * already passed 0 by the time the handler writes, and the match would be
 * missed -- so the two degenerate slices get one extra tick.  They are the
 * fully-off and fully-on cases, where the pin does not move anyway. */
ISR(TIMER2_COMP_vect)
{
    static uint8_t on_slice;
    uint8_t d = s_duty;

    if (on_slice) {                     /* the ON slice just expired */
        on_slice = 0;
        if (d < 255) LED_PORT &= (uint8_t)~(1 << LED_BIT);
        OCR2 = (uint8_t)((d < 254) ? (255 - d) : 1);
    } else {
        on_slice = 1;
        if (d) LED_PORT |= (uint8_t)(1 << LED_BIT);
        OCR2 = (uint8_t)((d > 1) ? (d - 1) : 1);
    }
}
