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

/* small integer helpers shared across modules */
uint16_t  isqrt32(uint32_t n);
uint16_t  median_u16(uint16_t *src, uint8_t n);   /* copies, non-destructive-ish */

#endif
