#include "power.h"
#include "sys.h"
#include "led.h"
#include "i2c.h"
#include "ssd1306.h"
#include "max30102.h"
#include "ppg.h"
#include "ui.h"
#include "settings.h"
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

/* The wake interrupt only has to break power-down; it must disable itself
 * immediately because a LEVEL-triggered interrupt keeps firing for as long
 * as the button is held down. */
ISR(INT0_vect)
{
    GICR &= (uint8_t)~(1 << INT0);
}

static void wink(uint8_t times)
{
    uint8_t i;
    for (i = 0; i < times; i++) {
        led_duty(200); _delay_ms(45); wdt_reset();
        led_duty(0);  _delay_ms(75); wdt_reset();
    }
}

static uint8_t btn_down_raw(void)
{
    return (uint8_t)((BTN_PIN & (1 << BTN_BIT)) ? 0 : 1);
}

/* Bounded wait for the button to be let go, so the press that triggered
 * (or ended) the sleep does not also count as a UI gesture. */
static void wait_release(uint16_t max_ms)
{
    uint32_t t0 = millis();
    while (btn_down_raw() && (uint32_t)(millis() - t0) < max_ms) wdt_reset();
    _delay_ms(BTN_DEBOUNCE_MS + 5);
    btn_flush();
}

void power_sleep(void)
{
    wait_release(3000);
    wink(2);                                /* "going to sleep" */

    /* ---------------- power everything down ---------------- */
    led_stop();
    /* Unconditional, and belt-and-braces: zeroing the drive currents makes
     * the emitters dark even if the mode write does not land, and this must
     * not be skipped just because a read failed earlier and cleared the
     * present flag -- that is exactly when the LEDs would be left on. */
    max30102_set_leds(0, 0);
    max30102_shutdown(1);
    ssd1306_power(0);                       /* display off      */
    ssd1306_charge_pump(0);                 /* and its DC-DC    */

    TWCR = 0;                               /* release the TWI  */
    SPCR = 0;                               /* SPI off          */

    TIMSK &= (uint8_t)~(1 << OCIE0);        /* stop the 1 ms tick */
    TCCR0 = 0;

    wdt_disable();                          /* or it resets us in 2 s */

    /* ---------------- arm the wake source ---------------- */
    cli();
    MCUCR &= (uint8_t)~((1 << ISC01) | (1 << ISC00));   /* low level */
    GIFR   = (uint8_t)(1 << INTF0);
    GICR  |= (uint8_t)(1 << INT0);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();
    sleep_cpu();                            /* ---- asleep here ---- */
    sleep_disable();

    GICR &= (uint8_t)~(1 << INT0);

    /* ---------------- bring the board back up ---------------- */
    TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);   /* 1 ms tick */
    OCR0  = 249;
    TIMSK |= (1 << OCIE0);
    sei();

    wdt_enable(WDTO_2S);

    spi_init();
    ssd1306_init();
    ssd1306_contrast(cfg.contrast);
    ssd1306_flip(cfg.flip);

    i2c_init();
    if (max30102_present()) {
        max30102_shutdown(0);
        max30102_flush_fifo();
    } else {
        max30102_init();                    /* it may have been unplugged */
    }
    if (max30102_present()) max30102_set_avg(cfg.avg_code);

    led_init();
    wink(1);                                /* "awake" */

    ppg_init();
    ui_wake();
    wait_release(4000);
}
