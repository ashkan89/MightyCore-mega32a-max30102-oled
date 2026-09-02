/* ------------------------------------------------------------------
 *  config.h -- board / build configuration
 *  Target : ATmega32A @ 16 MHz external crystal (MightyCore std pinout)
 * ------------------------------------------------------------------ */
#ifndef CONFIG_H
#define CONFIG_H

#include <avr/io.h>
#include <stdint.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* ---------------- Firmware identity ---------------- */
#define FW_NAME     "PulseOx"
#define FW_VERSION  "1.1.0"

/* ---------------- Flash and SRAM budget ----------------
 * 32 KB of flash, of which the top BOOTLOADER_RESERVE bytes belong to
 * urboot, and 2 KB of SRAM shared between static allocation and the call
 * stack.  Both budgets are enforced at build time by
 * scripts/check_size.py, because avr-size measures against the bare part
 * and so never warns about either.  Change the numbers there (or in
 * platformio.ini), not here -- these two lines only document them.
 *
 *   BOOTLOADER_RESERVE  512 B, the usual urboot footprint for this part
 *   STACK_RESERVE       224 B, against a measured worst case of 182 B
 * (the deepest chain is main -> max30102_read -> ppg_process ->
 *  median_u16, plus one interrupt frame; see README, "Stack")
 */

/* ---------------- SSD1306 OLED (hardware SPI, PORTB) ---------------- */
/*  SCK  = PB7  (hardware)              MOSI = PB5 (hardware)          */
/*  MISO = PB6  (unused, kept input)                                    */
/*  PB4 is the hardware /SS.  It is not the chip select on this board,  */
/*  so spi_init() holds it high with a pull-up -- see SPI_SS below.     */
#define OLED_PORT   PORTB
#define OLED_DDR    DDRB
#define OLED_CS     PB1
#define OLED_DC     PB2
#define OLED_RST    PB3
#define SPI_MOSI    PB5
#define SPI_MISO    PB6
#define SPI_SCK     PB7
#define SPI_SS      PB4        /* hardware /SS: must never float low */

/* ---------------- MAX30102 (TWI / I2C, PORTC) ---------------- */
/*  SCL = PC0, SDA = PC1  (hardware TWI on ATmega32)             */

/* ---------------- User button ---------------- */
#define BTN_PORT    PORTD
#define BTN_DDR     DDRD
#define BTN_PIN     PIND
#define BTN_BIT     PD2        /* active LOW, internal pull-up   */

/* ---------------- Optional piezo buzzer on OC1A = PD5 ---------------- */
#define USE_BUZZER  0          /* set to 1 if a buzzer is fitted */
#define BUZ_PORT    PORTD
#define BUZ_DDR     DDRD
#define BUZ_BIT     PD5

/* ---------------- Human interface timing (ms) ---------------- */
#define BTN_DEBOUNCE_MS   15
#define BTN_LONG_MS      600
#define BTN_DBL_MS       320

/* ---------------- Display ---------------- */
#define OLED_W       128
#define OLED_H        64
#define UI_FPS_MS     50       /* 20 fps redraw */

/* ---------------- Diagnostics ---------------- */
/*  UART0 TX on PD1 -- the same pin the urboot bootloader talks over, so a
 *  host that can flash the board can already read this.
 *
 *  DBG_MODE selects what goes out, and the three modes are mutually
 *  exclusive so their flash costs do not add up:
 *
 *    0  off.  Nothing is compiled in -- no UART setup, no strings, no
 *       probe.  Worth about 3 KB of flash, which is what makes room under
 *       the bootloader.  Use this for production; see env:release.
 *
 *    1  human-readable "name=value" status, one line every 2 s, plus one
 *       line per detected beat.  This is the default and what every fault
 *       in this firmware so far was found with.  Fields are documented in
 *       dbg_service().
 *
 *    2  CSV, one record per detected beat, with a header line at start-up.
 *       For capturing datasets to tune against instead of guessing; see
 *       README, "Capturing data".  Import straight into a spreadsheet.
 *
 *  Overridden per environment in platformio.ini, so leave this at 1 and
 *  build `pio run -e release` / `-e csv` rather than editing it.
 */
#ifndef DBG_MODE
#define DBG_MODE     1
#endif
/*  The periodic 2-second status line, separately from the mode.  It is
 *  the bulk of the diagnostic flash cost -- roughly 1.3 KB, most of it
 *  the field-name strings -- and the one-shot probe build does not need
 *  it: that build exists to run dbg_channel_probe() once and print its
 *  verdict, and it cannot afford both.  Only meaningful with DBG_MODE 1.
 */
#ifndef DBG_STATUS
#define DBG_STATUS   1
#endif

/*  Derived: any non-zero mode needs the UART. */
#define DBG_UART     (DBG_MODE != 0)
#define DBG_BAUD     38400UL

/*  Stack high-water mark.  The unused SRAM between the end of .bss and
 *  the initial stack pointer is painted with a known byte at start-up and
 *  the deepest point the stack ever reached is reported afterwards.  This
 *  is the only way to know the real margin on a part with 2 KB of RAM and
 *  a 1 KB framebuffer in it -- a static estimate cannot see interrupt
 *  frames or what the optimiser did to them.  Costs about 40 bytes of
 *  flash and no SRAM at all.  See sys_stack_free(). */
#ifndef STACK_GUARD
#define STACK_GUARD  1
#endif
/*  One-shot LED/FIFO channel identification at start-up, printed over the
 *  same UART -- see dbg_channel_probe().  It answers, from the hardware
 *  rather than from the datasheet, which word of each FIFO sample belongs to
 *  which emitter, because a part that disagrees inverts every SpO2 ratio.
 *
 *  The verdict no longer costs 5 s of every boot: it is cached in EEPROM
 *  the first time it is reached and reused afterwards, so the probe runs
 *  on a virgin board, after a factory reset, or when the sensor's part ID
 *  changes -- and never again in between.  Set to 0 to compile it out. */
#ifndef DBG_LED_PROBE
#define DBG_LED_PROBE 1
#endif

/* ---------------- Sensor / DSP ---------------- */
#define PPG_FS_NOM   100u      /* nominal sample rate after averaging */
#define WAVE_LEN     128       /* pixels of live waveform history     */
#define TREND_LEN     64       /* trend points (1 per 2 s = 128 s)    */

#endif /* CONFIG_H */
