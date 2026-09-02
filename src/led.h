/* led.h -- status LED on the MightyCore built-in pin (PB0), driven by an
 *          8-bit software PWM off Timer2 so it can breathe smoothly.
 *
 * PB0 has no timer output on the ATmega32 (OC0=PB3, OC1A=PD5, OC1B=PD4,
 * OC2=PD7), so the duty cycle has to be applied in software.  Rather than
 * counting PWM steps in an overflow ISR -- which needs one interrupt per step
 * and so trades resolution against CPU -- Timer2 runs in CTC and the ISR
 * reloads OCR2 with the remaining slice, toggling the pin once each time.
 * That is TWO interrupts per PWM period regardless of resolution:
 *
 *   16 MHz / 64 = 250 kHz, period = 256 ticks = 977 Hz refresh (flicker-free)
 *   256 duty levels, ~1950 interrupts/s, well under 1 % of the CPU
 *
 * The old version had 32 levels at 7.8 kHz, and 32 levels is simply not
 * enough to fade smoothly: near full brightness consecutive table entries
 * differed by 4/32, which reads as a staircase rather than a fade.
 */
#ifndef LED_H
#define LED_H
#include "config.h"

#define LED_PWM_MAX 255         /* led_duty() takes 0..LED_PWM_MAX */

enum {
    LED_OFF = 0,
    LED_BREATHE,     /* idle, waiting for a finger                  */
    LED_SEARCH,      /* finger present, still acquiring             */
    LED_BEAT,        /* locked: dim, with a flash on every heartbeat */
    LED_COUNTDOWN    /* about to sleep                              */
};

void led_init(void);
void led_stop(void);            /* silences Timer2 and parks the pin low */
void led_mode(uint8_t m);
void led_flash(void);           /* one-shot bright pulse (a heartbeat)   */
void led_service(void);         /* call from the main loop               */
void led_duty(uint8_t d);       /* 0..LED_PWM_MAX, direct control        */

#endif
