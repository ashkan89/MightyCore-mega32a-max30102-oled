/* ------------------------------------------------------------------
 *  PulseOx -- MAX30102 pulse oximeter for ATmega32A @ 16 MHz
 *
 *  OLED  : SSD1306 128x64, hardware SPI on PORTB
 *  Sensor: MAX30102 on TWI (PC0 = SCL, PC1 = SDA)
 *  Input : one button on PD2 (active low)
 *
 *  The main loop is cooperative and never blocks: FIFO drain, DSP,
 *  housekeeping and a 20 fps redraw, all under a 2 s watchdog.
 * ------------------------------------------------------------------ */
#include "config.h"
#include "sys.h"
#include "i2c.h"
#include "ssd1306.h"
#include "gfx.h"
#include "max30102.h"
#include "ppg.h"
#include "ui.h"
#include "settings.h"
#include "buzzer.h"
#include "led.h"
#include "power.h"
#include "dbg.h"
#include <avr/wdt.h>
#include <avr/interrupt.h>

/* Clear the watchdog as early as possible: a WDT reset leaves it running
 * with the shortest timeout on some AVRs, which would otherwise loop.  */
uint8_t mcusr_mirror __attribute__((section(".noinit")));
void early_init(void) __attribute__((naked, used, section(".init3")));
void early_init(void)
{
    mcusr_mirror = MCUCSR;
    MCUCSR = 0;
    wdt_disable();
}

static void sensor_bring_up(void)
{
    uint8_t tries = 0;
    while (!max30102_init()) {
        ui_sensor_error();
        wdt_reset();
        {   /* ~1 s of waiting, watchdog-friendly */
            uint32_t t = millis();
            while ((uint32_t)(millis() - t) < 1000) wdt_reset();
        }
        if (++tries >= 3) { tries = 0; i2c_recover(); }
    }
}

/* The sensor produces 100 samples/s (400 Hz with 4x averaging), one every
 * 10 ms, and the FIFO holds 32.  Polling it every pass of a 250 Hz loop was
 * three register reads per pass for nothing most of the time; every 4 ms is
 * still twice as often as samples arrive, and it leaves the loop free to run
 * fast for the button, the display and the status LED. */
#define FIFO_POLL_MS 4

int main(void)
{
    uint32_t last_draw = 0, last_probe = 0, last_fifo = 0;

    sys_init();
    dbg_init();
    spi_init();
    ssd1306_init();
    buz_init();
    led_init();

    settings_load();
    ssd1306_contrast(cfg.contrast);
    ssd1306_flip(cfg.flip);

    ui_splash();

    i2c_init();
    wdt_enable(WDTO_2S);
    sensor_bring_up();

    max30102_set_avg(cfg.avg_code);
    /* Identify the FIFO channel order against the hardware before the DSP
     * starts trusting it.  Costs about 1.7 s of start-up and prints its
     * verdict on the diagnostic UART; set DBG_LED_PROBE to 0 in config.h
     * once the answer is known.  ppg_init() below reprograms the drives. */
#if DBG_LED_PROBE
    ui_probe_result(dbg_channel_probe());
#endif
    ppg_init();
    ui_init();
    btn_flush();

    for (;;) {
        uint32_t now = millis();
        wdt_reset();
        dbg_loop();

        if (max30102_present()) {
            if ((uint32_t)(now - last_fifo) >= FIFO_POLL_MS) {
                last_fifo = now;
                /* Drain at most 8 samples per pass: two poll intervals'
                 * worth of headroom, and bounded so one pass cannot hold
                 * the loop long enough to stutter the display. */
                max30102_read(ppg_process, 8);
                ppg_lost_samples(max30102_take_ovf());
            }
            ppg_service();
        } else {
            if ((uint32_t)(now - last_probe) >= 1000) {
                last_probe = now;
                i2c_recover();
                if (max30102_init()) {
                    max30102_set_avg(cfg.avg_code);
                    ppg_init();
                }
            }
        }

        ui_event(btn_get());
        ui_tick();
        buz_service();
        dbg_service();
        led_service();

        if (ui_sleep_pending()) {
            power_sleep();          /* returns once the button wakes us */
            last_draw = last_probe = last_fifo = millis();
            continue;
        }

        now = millis();
        if ((uint32_t)(now - last_draw) >= UI_FPS_MS) {
            last_draw = now;
            if (max30102_present()) {
                ui_draw();
                ssd1306_flush();
            } else {
                ui_sensor_error();
            }
        }
    }
}
