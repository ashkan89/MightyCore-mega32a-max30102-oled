#include "sys.h"
#include <avr/interrupt.h>
#include <string.h>

static volatile uint32_t g_ms;

/* ------------------------------------------------------------------
 *  Reset cause and stack high-water mark
 * ------------------------------------------------------------------ */
uint8_t sys_mcucsr __attribute__((section(".noinit")));

char sys_reset_cause_ch(void)
{
    /* Checked most-serious-first: a watchdog reset alongside a power-on
     * flag means the watchdog fired, and that is the one worth knowing
     * about.  Several flags can be set at once because they are only
     * cleared by software. */
    if (sys_mcucsr & (1 << WDRF))  return 'W';   /* watchdog          */
    if (sys_mcucsr & (1 << BORF))  return 'B';   /* brown-out         */
    if (sys_mcucsr & (1 << EXTRF)) return 'E';   /* external / reset  */
    if (sys_mcucsr & (1 << PORF))  return 'P';   /* power-on          */
    return '?';
}

#if STACK_GUARD

/* The paint byte.  0xC5 is not 0x00 or 0xFF, so it cannot be confused
 * with erased RAM or with a cleared .bss, and it is not a plausible
 * return address byte either. */
#define STACK_PAINT 0xC5

extern uint8_t _end;          /* first byte past .bss, from the linker */

/* Runs from .init1, which is the earliest useful point: before
 * __do_copy_data and __do_clear_bss in .init4, so neither can undo the
 * paint, and before any C code has run, so the whole region above .bss is
 * genuinely unused.  Naked, because a prologue would itself push onto the
 * region being painted.
 *
 * The upper bound is the LINKER's __stack (RAMEND), not the stack
 * pointer.  That distinction matters and is not cosmetic: .init1 runs
 * before .init2, which is where avr-libc loads SP from __stack, and SP
 * resets to 0x0000 on this part.  Reading SP here therefore yields 0, and
 * a loop bounded by "SP minus a guard" would run from _end upwards
 * through 0xFFF8 -- straight over the whole of SRAM and the I/O register
 * file.  Using the symbol makes the bound a compile-time constant that
 * cannot depend on machine state this early.
 *
 * Painting through RAMEND itself is harmless: .init2 sets SP = RAMEND and
 * the first push decrements before storing, and in any case
 * sys_stack_free() counts upward from _end and never looks at the top.
 *
 * Registers: r24 and X/Z only.  Nothing is initialised yet -- r1 is not
 * cleared until .init2 -- so nothing may be assumed about any register,
 * and in particular r1 is left untouched.  */
void sys_stack_paint(void) __attribute__((naked, used, section(".init1")));
void sys_stack_paint(void)
{
    __asm__ volatile (
        "    ldi  r26, lo8(%1)      \n"   /* X = RAMEND + 1, the limit  */
        "    ldi  r27, hi8(%1)      \n"
        "    ldi  r30, lo8(%0)      \n"   /* Z = &_end, first free byte */
        "    ldi  r31, hi8(%0)      \n"
        "    ldi  r24, %2           \n"   /* the paint byte             */
        "1:  st   Z+, r24           \n"
        "    cp   r30, r26          \n"
        "    cpc  r31, r27          \n"
        "    brlo 1b                \n"   /* while Z < limit            */
        :: "i" (&_end), "i" (RAMEND + 1), "M" (STACK_PAINT)
        : "r24", "r26", "r27", "r30", "r31"
    );
}

uint16_t sys_stack_free(void)
{
    const uint8_t *p = &_end;
    uint16_t n = 0;
    /* Count the paint still standing from the bottom up.  The first byte
     * that is not paint is as deep as the stack has ever been.  A false
     * positive is possible in principle -- a live stack byte that happens
     * to equal the paint value -- but it can only ever make the answer
     * look worse than it is, which is the safe direction. */
    while (p < (const uint8_t *)RAMEND && *p == STACK_PAINT) { p++; n++; }
    return n;
}

#else
uint16_t sys_stack_free(void) { return 0xFFFF; }
#endif

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
