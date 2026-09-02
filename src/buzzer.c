#include "buzzer.h"
#include "sys.h"

#if USE_BUZZER

static uint32_t s_off_ms;
static uint8_t  s_on;

void buz_init(void)
{
    BUZ_DDR  |= (uint8_t)(1 << BUZ_BIT);
    BUZ_PORT &= (uint8_t)~(1 << BUZ_BIT);
    TCCR1A = 0;
    TCCR1B = 0;
}

void buz_beep(uint16_t hz, uint16_t ms)
{
    uint32_t top;
    if (!hz || !ms) return;
    top = (F_CPU / 8UL) / (2UL * hz);          /* prescaler 8, toggle OC1A */
    if (top < 1) top = 1;
    if (top > 65535UL) top = 65535UL;
    OCR1A  = (uint16_t)top;
    TCNT1  = 0;
    TCCR1A = (1 << COM1A0);                    /* toggle OC1A on compare  */
    TCCR1B = (1 << WGM12) | (1 << CS11);       /* CTC, clk/8              */
    s_off_ms = millis() + ms;
    s_on = 1;
}

void buz_service(void)
{
    if (s_on && (int32_t)(millis() - s_off_ms) >= 0) {
        TCCR1A = 0;
        TCCR1B = 0;
        BUZ_PORT &= (uint8_t)~(1 << BUZ_BIT);
        s_on = 0;
    }
}

#else   /* buzzer not fitted: keep the call sites clean */

void buz_init(void)                        { }
void buz_beep(uint16_t hz, uint16_t ms)    { (void)hz; (void)ms; }
void buz_service(void)                     { }

#endif
