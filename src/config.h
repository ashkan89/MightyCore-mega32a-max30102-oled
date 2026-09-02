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
#define FW_VERSION  "1.0.0"

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
 *  host that can flash the board can already read this.  One
 *  "name=value" line per second; see dbg.c.  Set to 0 to compile it out. */
#define DBG_UART     1
#define DBG_BAUD     38400UL
/*  One-shot LED/FIFO channel identification at start-up, printed over the
 *  same UART -- see dbg_channel_probe().  It answers, from the hardware
 *  rather than from the datasheet, which word of each FIFO sample belongs to
 *  which emitter, because a part that disagrees inverts every SpO2 ratio.
 *  Adds about 1.7 s to boot, so turn it off once the answer is known. */
#define DBG_LED_PROBE 1

/* ---------------- Sensor / DSP ---------------- */
#define PPG_FS_NOM   100u      /* nominal sample rate after averaging */
#define WAVE_LEN     128       /* pixels of live waveform history     */
#define TREND_LEN     64       /* trend points (1 per 2 s = 128 s)    */

#endif /* CONFIG_H */
