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
 * with the shortest timeout on some AVRs, which would otherwise loop.
 *
 * MCUCSR is read into sys_mcucsr on the way past and then zeroed, because
 * the reset flags are sticky -- left alone, the next reset would report
 * this one's cause on top of its own and "why did it reboot" becomes
 * unanswerable.  sys_reset_cause_ch() decodes it and the sensor screen
 * and the diagnostic line both show it.  */
void early_init(void) __attribute__((naked, used, section(".init3")));
void early_init(void)
{
    sys_mcucsr = MCUCSR;
    MCUCSR = 0;
    wdt_disable();
}

/* Bring the sensor up, but do NOT stand here for ever if it will not
 * answer.
 *
 * This used to loop until max30102_init() succeeded, which meant a board
 * with an unplugged or dead sensor never reached the main loop at all: no
 * screens, no menu, no auto-sleep, no way to see the firmware version --
 * just the fault screen, permanently.  The main loop already handles an
 * absent sensor properly (it retries every second, re-initialises on
 * success and draws the same fault screen), so the right thing to do
 * after a few honest attempts is to hand over to it.  Everything
 * downstream tolerates a sensor that is not present: ppg_init() only
 * writes registers, and the writes fail harmlessly. */
#define BRINGUP_ATTEMPTS 3

static void sensor_bring_up(void)
{
    uint8_t tries;
    for (tries = 0; tries < BRINGUP_ATTEMPTS; tries++) {
        if (max30102_init()) return;
        ui_sensor_error();
        {   /* ~1 s of waiting, watchdog-friendly */
            uint32_t t = millis();
            while ((uint32_t)(millis() - t) < 1000) wdt_reset();
        }
        i2c_recover();
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

    /* ---- FIFO channel order ----
     * Which 3-byte word of each sample belongs to which emitter is
     * established against the hardware rather than assumed, because a part
     * that disagrees inverts every SpO2 ratio into 1/R.
     *
     * The verdict is applied from EEPROM FIRST and unconditionally.  It has
     * to be: a release build has no probe compiled in at all, and would
     * otherwise silently drop a correction that had already been
     * established on this very board -- reintroducing the exact fault the
     * probe exists to catch, in the one build where it matters most.
     */
#if SENSOR_WORD_ORDER >= 0
    /* Told explicitly, so neither the probe nor the EEPROM gets a say.
     * See SENSOR_WORD_ORDER in config.h. */
    max30102_set_word_order(SENSOR_WORD_ORDER);
#else
    if (cfg.probe_v != PROBE_UNKNOWN)
        max30102_set_word_order((uint8_t)(cfg.probe_v == PROBE_REVERSED));
#endif

#if DBG_LED_PROBE && SENSOR_WORD_ORDER < 0
    /* Then probe only if the cached answer does not apply: never
     * established, or established against a different part.  The probe
     * costs about 5 s of start-up between the three optical passes and the
     * result screen, and the answer cannot change while the same sensor is
     * fitted, so paying it on every boot for the life of the device was
     * simply waste.  A factory reset clears the cache and re-arms it.
     *
     * Skipped entirely if the sensor is absent: the probe would measure
     * three sets of failed reads and conclude nonsense from them. */
    if (max30102_present() &&
        (cfg.probe_v == PROBE_UNKNOWN || cfg.probe_id != max30102_part_id())) {
        uint8_t v = dbg_channel_probe();     /* sets the word order itself */
        /* Only a CONCLUSIVE verdict is cached.  NORED, NOIR and BOTH all
         * mean the probe could not tell -- the result screen says to retry
         * with a finger on and out of bright light -- and caching one of
         * those made retrying impossible: the cache is no longer
         * PROBE_UNKNOWN, so the probe never runs again, while
         * `probe_v == PROBE_REVERSED` stays false and the datasheet order
         * is left in place for good.  One inconclusive probe therefore
         * used to lock in the wrong channel order permanently, which is
         * exactly the fault the probe exists to prevent.  Leave the cache
         * alone instead, so the next boot tries again. */
        if (v == PROBE_OK || v == PROBE_REVERSED) {
            cfg.probe_v  = v;
            cfg.probe_id = max30102_part_id();
            settings_save();                 /* once per sensor, not per boot */
        }
        ui_probe_result(v);
    }
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
