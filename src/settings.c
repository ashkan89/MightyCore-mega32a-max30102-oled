#include "settings.h"
#include "ssd1306.h"
#include "max30102.h"
#include <avr/eeprom.h>

settings_t cfg;

static settings_t EEMEM ee_cfg;

static uint8_t crc8(const uint8_t *p, uint8_t n)
{
    uint8_t c = 0xFF, i, b;
    for (i = 0; i < n; i++) {
        c ^= p[i];
        for (b = 0; b < 8; b++)
            c = (uint8_t)((c & 0x80) ? ((c << 1) ^ 0x31) : (c << 1));
    }
    return c;
}

void settings_defaults(void)
{
    cfg.magic     = 0xA5;
    cfg.version   = 3;
    cfg.contrast  = 0xCF;
    cfg.flip      = 0;
    cfg.beep      = 0;
    cfg.led_mode  = LED_MODE_AUTO;
    cfg.avg_code  = 2;          /* 4x -> 100 Hz effective */
    cfg.spo2_cal  = 0;
    cfg.dim_s     = 0;
    cfg.sleep_s   = 120;
    cfg.start_scr = 0;
    cfg.fs_cal    = 0;          /* learn it on the first run */
}

void settings_load(void)
{
    eeprom_read_block(&cfg, &ee_cfg, sizeof(cfg));
    if (cfg.magic != 0xA5 || cfg.version != 3 ||
        cfg.crc != crc8((const uint8_t *)&cfg, (uint8_t)(sizeof(cfg) - 1))) {
        settings_defaults();
        settings_save();
    }
    if (cfg.avg_code > 5)  cfg.avg_code = 2;
    if (cfg.led_mode > 3)  cfg.led_mode = LED_MODE_AUTO;
    if (cfg.spo2_cal > 50) cfg.spo2_cal = 50;
    if (cfg.spo2_cal < -50) cfg.spo2_cal = -50;
    if (cfg.sleep_s > 600) cfg.sleep_s = 120;
    /* A stored rate outside the plausible span means a corrupt or foreign
     * record; fall back to learning it again. */
    if (cfg.fs_cal && (cfg.fs_cal < 3000 || cfg.fs_cal > 60000)) cfg.fs_cal = 0;
}

void settings_save(void)
{
    cfg.magic   = 0xA5;
    cfg.version = 3;
    cfg.crc     = crc8((const uint8_t *)&cfg, (uint8_t)(sizeof(cfg) - 1));
    eeprom_update_block(&cfg, &ee_cfg, sizeof(cfg));
}

void settings_apply(void)
{
    ssd1306_contrast(cfg.contrast);
    ssd1306_flip(cfg.flip);
    if (max30102_present()) max30102_set_avg(cfg.avg_code);
}
