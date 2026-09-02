#include "sys.h"
#include <avr/interrupt.h>
#include <string.h>

static volatile uint32_t g_ms;

/* ---- button state machine (runs inside the 1 ms tick) ---- */
static volatile uint8_t  g_btn_stable;      /* 1 = pressed */
static uint8_t  s_raw_prev, s_deb_cnt;
static uint8_t  s_long_sent, s_click_pending;
static uint32_t s_press_t, s_click_t;

/* 4-deep event ring */
static volatile btn_evt_t s_q[4];
static volatile uint8_t   s_qh, s_qt;

static void q_push(btn_evt_t e)
{
    uint8_t n = (uint8_t)((s_qh + 1) & 3);
    if (n != s_qt) { s_q[s_qh] = e; s_qh = n; }
}

void sys_init(void)
{
    /* Timer0: CTC, prescaler 64, OCR0 = 249 -> 16 MHz/64/250 = 1000 Hz */
    TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);
    OCR0  = 249;
    TIMSK |= (1 << OCIE0);

    /* button input with pull-up */
    BTN_DDR  &= (uint8_t)~(1 << BTN_BIT);
    BTN_PORT |=  (uint8_t)(1 << BTN_BIT);


    sei();
}

uint32_t millis(void)
{
    uint32_t v;
    uint8_t s = SREG; cli();
    v = g_ms;
    SREG = s;
    return v;
}

btn_evt_t btn_get(void)
{
    btn_evt_t e = BTN_NONE;
    uint8_t s = SREG; cli();
    if (s_qt != s_qh) { e = s_q[s_qt]; s_qt = (uint8_t)((s_qt + 1) & 3); }
    SREG = s;
    return e;
}

void btn_flush(void)
{
    uint8_t s = SREG; cli();
    s_qt = s_qh; s_click_pending = 0;
    SREG = s;
}

ISR(TIMER0_COMP_vect)
{
    uint32_t now = ++g_ms;

    /* ---- debounce ---- */
    uint8_t raw = (uint8_t)((BTN_PIN & (1 << BTN_BIT)) ? 0 : 1);
    if (raw != s_raw_prev) { s_raw_prev = raw; s_deb_cnt = 0; }
    else if (s_deb_cnt < BTN_DEBOUNCE_MS) {
        if (++s_deb_cnt == BTN_DEBOUNCE_MS && raw != g_btn_stable) {
            g_btn_stable = raw;
            if (raw) {                       /* ---- press edge ---- */
                s_press_t   = now;
                s_long_sent = 0;
            } else {                         /* ---- release edge --- */
                if (!s_long_sent) {
                    if (s_click_pending && (uint16_t)(now - s_click_t) < BTN_DBL_MS) {
                        s_click_pending = 0;
                        q_push(BTN_DOUBLE);
                    } else {
                        s_click_pending = 1;
                        s_click_t = now;
                    }
                }
            }
        }
    }

    /* ---- long press while held ---- */
    if (g_btn_stable && !s_long_sent && (uint16_t)(now - s_press_t) >= BTN_LONG_MS) {
        s_long_sent     = 1;
        s_click_pending = 0;
        q_push(BTN_LONG);
    }
    /* ---- single click matures once the double window closes ---- */
    if (s_click_pending && (uint16_t)(now - s_click_t) >= BTN_DBL_MS) {
        s_click_pending = 0;
        q_push(BTN_CLICK);
    }
}

/* ------------------------------------------------------------------ */
uint16_t isqrt32(uint32_t n)
{
    uint32_t r = 0, b = 1UL << 30;
    while (b > n) b >>= 2;
    while (b) {
        if (n >= r + b) { n -= r + b; r = (r >> 1) + b; }
        else            { r >>= 1; }
        b >>= 2;
    }
    return (uint16_t)r;
}

uint16_t median_u16(uint16_t *src, uint8_t n)
{
    uint16_t t[12];
    uint8_t i, j;
    if (n == 0) return 0;
    if (n > 12) n = 12;
    memcpy(t, src, (size_t)n * 2);
    for (i = 1; i < n; i++) {                 /* insertion sort */
        uint16_t k = t[i];
        for (j = i; j && t[j - 1] > k; j--) t[j] = t[j - 1];
        t[j] = k;
    }
    return t[n >> 1];
}
