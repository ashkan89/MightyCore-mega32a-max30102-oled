/* sys.h -- 1 ms system tick, millis(), debounced single-button event queue */
#ifndef SYS_H
#define SYS_H
#include "config.h"

typedef enum {
    BTN_NONE = 0,
    BTN_CLICK,      /* short press                */
    BTN_DOUBLE,     /* two short presses          */
    BTN_LONG        /* held > BTN_LONG_MS         */
} btn_evt_t;

void      sys_init(void);
uint32_t  millis(void);
btn_evt_t btn_get(void);      /* pops one event, BTN_NONE if queue empty */
void      btn_flush(void);

/* ---- reset cause ----
 * MCUCSR is latched and cleared in .init3 before anything can disturb it
 * (see early_init() in main.c), because the flags are sticky: leave them
 * and the next reset reports this one's cause as well as its own.  A
 * device that has been running for days and quietly rebooted looks
 * identical to one that was just plugged in unless this is recorded.
 * Bits are PORF, EXTRF, BORF, WDRF -- see sys_reset_cause_ch(). */
extern uint8_t sys_mcucsr;    /* raw MCUCSR as it was at reset */
char      sys_reset_cause_ch(void);  /* 'P','E','B','W' or '?' */

/* ---- stack high-water mark ----
 * sys_stack_free() returns the number of bytes between the end of .bss
 * and the deepest point the stack has ever reached, by looking for the
 * paint applied in .init1.  0 means the paint is gone entirely, which is
 * a stack that has already collided with .bss -- the corruption has
 * happened, not merely become possible.  Returns 0xFFFF when the guard
 * is compiled out. */
uint16_t  sys_stack_free(void);

/* small integer helpers shared across modules */
uint16_t  isqrt32(uint32_t n);
uint16_t  median_u16(uint16_t *src, uint8_t n);   /* copies, non-destructive-ish */

#endif
