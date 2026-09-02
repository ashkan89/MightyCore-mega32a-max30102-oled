/* buzzer.h -- optional non-blocking piezo beeper on OC1A (PD5) */
#ifndef BUZZER_H
#define BUZZER_H
#include "config.h"

void buz_init(void);
void buz_beep(uint16_t hz, uint16_t ms);
void buz_service(void);          /* call from the main loop */

#endif
