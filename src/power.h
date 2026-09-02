/* power.h -- deep sleep and wake-on-button
 *
 * Everything that draws current is shut down in turn: LED PWM, MAX30102
 * (register-preserving shutdown, ~0.7 uA), the OLED panel and its charge
 * pump, SPI, TWI and the watchdog.  The MCU then enters power-down, where
 * only an asynchronous source can wake it -- on the ATmega32 that means a
 * LOW-LEVEL interrupt on INT0, which is exactly where the button sits (PD2).
 */
#ifndef POWER_H
#define POWER_H
#include "config.h"

/* Blocking. Returns once the button has woken the board and every
 * peripheral has been brought back up. */
void power_sleep(void);

#endif
