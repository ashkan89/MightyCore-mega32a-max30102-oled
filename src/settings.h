/* settings.h -- EEPROM-backed user settings with magic + CRC validation */
#ifndef SETTINGS_H
#define SETTINGS_H
#include "config.h"

#define LED_MODE_AUTO 0
#define LED_MODE_LOW  1
#define LED_MODE_MED  2
#define LED_MODE_HIGH 3

/* Cached start-up channel-probe verdict.  0xFF means "never established",
 * which is what a virgin EEPROM, a factory reset, or a swapped sensor
 * leaves behind and what makes the probe run again. */
#define PROBE_UNKNOWN 0xFF

#define SETTINGS_VERSION 4

typedef struct {
    uint8_t magic;      /* 0xA5                                     */
    uint8_t version;    /* SETTINGS_VERSION                         */
    uint8_t contrast;   /* OLED contrast 0..255                     */
    uint8_t flip;       /* 180 deg rotation                         */
    uint8_t beep;       /* beat beep (needs USE_BUZZER)             */
    uint8_t led_mode;   /* LED_MODE_*                               */
    uint8_t avg_code;   /* MAX30102 sample averaging 0..5           */
    int8_t  spo2_cal;   /* SpO2 trim, tenths of a percent, -50..+50 */
    uint8_t dim_s;      /* auto-dim after N s idle, 0 = never       */
    uint16_t sleep_s;   /* deep sleep after N s with no finger, 0=off*/
    uint8_t start_scr;  /* screen shown at power-up                 */
    uint16_t fs_cal;    /* measured ADC output rate x100, 0=unknown */
    /* Which averaging setting fs_cal was measured at.  The FIFO output
     * rate is 400 Hz / 2^avg_code, so a rate learned at one averaging
     * setting is wrong by up to 32x at another -- and beats are timed by
     * counting samples, so that error lands straight on the BPM.  Storing
     * it lets settings_load() throw the calibration away instead of
     * trusting it.  See ppg_init(). */
    uint8_t fs_cal_avg;
    uint8_t probe_v;    /* cached channel-probe verdict, PROBE_UNKNOWN  */
    uint8_t probe_id;   /* PART_ID the verdict was established against  */
    uint8_t crc;
} settings_t;

extern settings_t cfg;

void settings_defaults(void);
void settings_load(void);
void settings_save(void);
void settings_apply(void);      /* pushes contrast/flip/LED/avg to hardware */

#endif
