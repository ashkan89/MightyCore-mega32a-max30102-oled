/* ui.h -- screen carousel, settings menu and single-button interaction */
#ifndef UI_H
#define UI_H
#include "config.h"
#include "sys.h"

void ui_init(void);
void ui_splash(void);
void ui_sensor_error(void);      /* blocking retry screen */
void ui_probe_result(uint8_t verdict);  /* start-up channel-probe outcome */
void ui_event(btn_evt_t e);
void ui_tick(void);              /* trend sampling + housekeeping */
void ui_draw(void);              /* renders into fb; caller flushes */
uint8_t ui_sleep_pending(void);  /* 1 = idle countdown expired      */
void ui_wake(void);              /* called after power_sleep returns */

#endif
