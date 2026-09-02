#include "ui.h"
#include "gfx.h"
#include "ssd1306.h"
#include "ppg.h"
#include "max30102.h"
#include "settings.h"
#include "buzzer.h"
#include "led.h"
#include "i2c.h"
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>

/* ------------------------------------------------------------------ */
enum { SCR_MONITOR, SCR_WAVE, SCR_TREND, SCR_STATS, SCR_SENSOR, SCR_COUNT };
enum { MODE_SCREEN, MODE_MENU, MODE_EDIT, MODE_HELP, MODE_SLEEPWAIT };

#define SLEEP_COUNTDOWN_S 9

/* ------------------------------------------------------------------
 *  Layout for a two-colour panel
 *
 *  These 128x64 modules are not monochrome: the top 16 rows of glass are
 *  yellow and rows 16..63 are blue.  The split is in the panel itself, so
 *  it lands at the same place whatever is drawn -- which means the header
 *  has to be exactly 16 rows tall, not the 9 it used to be.  Anything else
 *  puts the first line or two of body text on yellow glass and cuts labels
 *  in half along a colour boundary.
 *
 *  So: rows 0..15 are the header -- bright text on black, ruled off at row
 *  15 -- and every screen's content starts at BODY_Y.  That leaves 47 rows,
 *  which is six text lines at ROW_H: ROW(0)..ROW(5) below.
 * ------------------------------------------------------------------ */
#define HDR_H    16            /* the panel's yellow band, exactly       */
#define BODY_Y   17            /* first usable blue row                  */
#define ROW_H     8            /* 7px font + 1                           */
#define ROW(n)   (BODY_Y + (n) * ROW_H)     /* 17 25 33 41 49 57         */

enum {
    MI_LED, MI_AVG, MI_CAL, MI_CONTRAST, MI_FLIP, MI_BEEP, MI_DIM, MI_SLEEP,
    MI_START, MI_RESET_SESSION, MI_FACTORY, MI_HELP, MI_SAVE, MI_COUNT
};

static uint8_t  s_mode, s_scr, s_sel, s_top;
static settings_t s_backup;
static uint32_t s_toast_ms;
static const char *s_toast;          /* PROGMEM pointer */
static uint32_t s_last_input_ms;
static uint8_t  s_dimmed;
static uint32_t s_idle_ms;          /* last finger contact or button press */
static uint8_t  s_mode_saved;
static uint8_t  s_sleep_req;
static uint8_t  s_countdown;

/* trend rings, one point every 2 s -> 128 s of history */
static uint8_t  tr_bpm[TREND_LEN];
static uint8_t  tr_spo2[TREND_LEN];
static uint8_t  tr_head;
static uint32_t tr_ms;

/* ------------------------------------------------------------------ */
static const char S_TITLE[]  PROGMEM = "PulseOx";
static const char S_MONITOR[]PROGMEM = "MONITOR";
static const char S_WAVE[]   PROGMEM = "WAVEFORM";
static const char S_TREND[]  PROGMEM = "TRENDS";
static const char S_STATS[]  PROGMEM = "ANALYSIS";
static const char S_SENSOR[] PROGMEM = "SENSOR";
static const char S_MENU[]   PROGMEM = "SETTINGS";
static const char S_DASH3[]  PROGMEM = "---";
static const char S_SAVED[]  PROGMEM = "SAVED";
static const char S_CLEARED[]PROGMEM = "SESSION CLEARED";
static const char S_DEFAULTS[]PROGMEM= "DEFAULTS LOADED";

static const char MN_LED[]  PROGMEM = "LED Drive";
static const char MN_AVG[]  PROGMEM = "Averaging";
static const char MN_CAL[]  PROGMEM = "SpO2 Trim";
static const char MN_CON[]  PROGMEM = "Contrast";
static const char MN_FLP[]  PROGMEM = "Flip 180";
static const char MN_BEP[]  PROGMEM = "Beat Beep";
static const char MN_DIM[]  PROGMEM = "Auto Dim";
static const char MN_SLP[]  PROGMEM = "Auto Sleep";
static const char MN_STA[]  PROGMEM = "Home Screen";
static const char MN_RST[]  PROGMEM = "Clear Session";
static const char MN_FAC[]  PROGMEM = "Factory Reset";
static const char MN_HLP[]  PROGMEM = "Controls";
static const char MN_SAV[]  PROGMEM = "Save & Exit";

static PGM_P const menu_names[MI_COUNT] PROGMEM = {
    MN_LED, MN_AVG, MN_CAL, MN_CON, MN_FLP, MN_BEP, MN_DIM, MN_SLP,
    MN_STA, MN_RST, MN_FAC, MN_HLP, MN_SAV
};

static const char V_AUTO[] PROGMEM = "Auto";
static const char V_LOW[]  PROGMEM = "Low";
static const char V_MED[]  PROGMEM = "Med";
static const char V_HIGH[] PROGMEM = "High";
static const char V_OFF[]  PROGMEM = "Off";
static const char V_ON[]   PROGMEM = "On";
static const char V_GO[]   PROGMEM = "hold";

static PGM_P const scr_names[SCR_COUNT] PROGMEM = {
    S_MONITOR, S_WAVE, S_TREND, S_STATS, S_SENSOR
};

static void hex8(int16_t x, int16_t y, uint8_t v);

/* ------------------------------------------------------------------ */
static void toast(const char *pstr)
{
    s_toast    = pstr;
    s_toast_ms = millis() + 1200;
}

void ui_init(void)
{
    s_mode = MODE_SCREEN;
    s_scr  = (uint8_t)(cfg.start_scr < SCR_COUNT ? cfg.start_scr : 0);
    s_sel  = s_top = 0;
    memset(tr_bpm, 0, sizeof(tr_bpm));
    memset(tr_spo2, 0, sizeof(tr_spo2));
    tr_head = 0;
    tr_ms   = millis();
    s_last_input_ms = s_idle_ms = millis();
    s_sleep_req = 0;
}

uint8_t ui_sleep_pending(void) { return s_sleep_req; }

void ui_wake(void)
{
    s_sleep_req = 0;
    s_dimmed    = 0;
    s_mode      = MODE_SCREEN;
    s_scr       = (uint8_t)(cfg.start_scr < SCR_COUNT ? cfg.start_scr : 0);
    memset(tr_bpm, 0, sizeof(tr_bpm));
    memset(tr_spo2, 0, sizeof(tr_spo2));
    tr_head = 0;
    tr_ms = s_last_input_ms = s_idle_ms = millis();
    btn_flush();
}

/* ============================ splash ============================ */
void ui_splash(void)
{
    uint8_t i;
    for (i = 0; i <= 16; i++) {
        gfx_clear();
        /* Name in the yellow band, everything else below it, so the splash
         * announces the panel's colour split instead of being bisected by
         * it -- scale-2 text is 14 rows and fits the 16-row band exactly. */
        gfx_text_P((int16_t)(64 - gfx_text_w_P(S_TITLE, 2) / 2), 1, S_TITLE, 2, C_WHITE);
        gfx_hline(0, HDR_H - 1, 128, C_WHITE);
        gfx_heart(58, 24, 1, C_WHITE);
        gfx_text(64 - gfx_text_w("v" FW_VERSION, 1) / 2, 42, "v" FW_VERSION, 1, C_WHITE);
        gfx_progress(14, 54, 100, 6, (uint8_t)(i * 100 / 16));
        ssd1306_flush();
        _delay_ms(45);
    }
    _delay_ms(250);
}

/* When the sensor will not answer, guessing is useless -- show what the bus
 * actually looks like: line levels (a LOW idle means missing pull-ups or a
 * short), who ACKs, and what came back from 0x57 if anything did. */
void ui_sensor_error(void)
{
    static uint32_t scan_ms;
    static uint8_t  found[5], nfound, lines, scanned;
    uint32_t now = millis();
    uint8_t  i;

    /* A full scan is 112 probes, and on a dead bus every one burns its whole
     * TWI timeout -- over two seconds in which the retry cannot run and the
     * screen does not update.  Rescan slowly, and once the scan has actually
     * turned something up there is nothing further to learn from repeating
     * it, so only the line levels are refreshed after that. */
    lines = i2c_lines();
    if (!scanned || (!nfound && (uint32_t)(now - scan_ms) >= 5000)) {
        scanned = 1;
        scan_ms = now;
        /* If a line is stuck low nothing can ACK, and every probe would just
         * burn its full timeout -- so do not bother scanning. */
        nfound  = (uint8_t)(((lines & 3) == 3) ? i2c_scan(found, 5) : 0);
    }

    gfx_clear();
    gfx_text_P(2, 1, PSTR("SENSOR FAULT"), 1, C_WHITE);
    gfx_num(74, 1, i2c_get_speed_khz(), 1, C_WHITE);
    gfx_text_P(92, 1, PSTR("kHz"), 1, C_WHITE);
    gfx_text_P(2, 9, PSTR("SCL"), 1, C_WHITE);
    gfx_text_P(22, 9, (lines & 1) ? PSTR("H") : PSTR("L"), 1, C_WHITE);
    gfx_text_P(40, 9, PSTR("SDA"), 1, C_WHITE);
    gfx_text_P(62, 9, (lines & 2) ? PSTR("H") : PSTR("L"), 1, C_WHITE);
    gfx_hline(0, HDR_H - 1, 128, C_WHITE);

    if ((lines & 3) != 3) {
        gfx_text_P(2, ROW(0), PSTR("LINE LOW: pull-ups?"), 1, C_WHITE);
        gfx_text_P(2, ROW(1), PSTR("or SDA/SCL shorted"), 1, C_WHITE);
    } else if (nfound == 0) {
        gfx_text_P(2, ROW(0), PSTR("no device on the bus"), 1, C_WHITE);
        gfx_text_P(2, ROW(1), PSTR("check VIN, GND, wires"), 1, C_WHITE);
    } else {
        gfx_text_P(2, ROW(0), PSTR("found"), 1, C_WHITE);
        gfx_num(34, ROW(0), nfound, 1, C_WHITE);
        gfx_text_P(44, ROW(0), PSTR("device(s):"), 1, C_WHITE);
        for (i = 0; i < nfound && i < 4; i++)
            hex8((int16_t)(2 + i * 32), ROW(1), found[i]);
    }

    if (max30102_id_acked()) {
        gfx_text_P(2, 43, PSTR("0x57 ID"), 1, C_WHITE);
        hex8(46, 43, max30102_part_id());
        gfx_text_P(76, 43, PSTR("need 15"), 1, C_WHITE);
    } else {
        gfx_text_P(2, 43, PSTR("0x57 did not answer"), 1, C_WHITE);
    }

    gfx_text_P(2, ROW(5), PSTR("retrying..."), 1, C_WHITE);
    if (s_mode == MODE_SLEEPWAIT) {
        gfx_text_P(68, ROW(5), PSTR("sleep in"), 1, C_WHITE);
        gfx_num(118, ROW(5), s_countdown, 1, C_WHITE);
    }
    ssd1306_flush();
}

/* ======================= shared chrome ======================= */
/* The header is two lines inside the yellow band: the screen name and a
 * quality bar on the first, and what the reading is currently doing on the
 * second.  Putting the state up here rather than in the body means the blue
 * area is only ever measurements, which is what makes the colour split read
 * as deliberate instead of accidental. */
/* The header is drawn as bright text on black, NOT as a filled block with
 * inverse text.  Filling all 16 rows lights 2048 pixels at once, and on these
 * modules a large solid area loads the panel's own charge pump hard enough to
 * sag its supply -- which shows up as the whole display flickering.  The 9-row
 * bar it replaced lit only half as many, so widening the band to match the
 * yellow glass is exactly what started it.  Bright text on black is also the
 * conventional look for a two-colour panel, and it draws a fraction of the
 * current: a rule at row 15 marks the boundary the glass already has. */
static void title_bar(PGM_P name)
{
    gfx_text_P(2, 1, name, 1, C_WHITE);
    gfx_hline(0, HDR_H - 1, 128, C_WHITE);

    if (ppg.finger) {
        uint8_t i, bars = (uint8_t)((ppg.sqi + 19) / 20);   /* 0..5 */
        for (i = 0; i < 5; i++) {
            int16_t x = (int16_t)(103 + i * 5);
            int16_t h = (int16_t)(2 + i);
            if (i < bars) gfx_fill(x, (int16_t)(8 - h), 3, h, C_WHITE);
            else          gfx_rect(x, (int16_t)(8 - h), 3, h, C_WHITE);
        }
        /* heart blinks for 160 ms after each detected beat */
        if ((uint32_t)(millis() - ppg.beat_ms) < 160)
            gfx_heart(90, 0, 0, C_WHITE);

        gfx_text_P(2, 9, ppg.valid ? PSTR("READY") : PSTR("ACQUIRING"), 1, C_WHITE);
        gfx_text_P(98, 9, PSTR("Q"), 1, C_WHITE);
        gfx_num(106, 9, ppg.sqi, 1, C_WHITE);
    } else {
        gfx_text_P(2, 9, PSTR("PLACE FINGER"), 1, C_WHITE);
    }
}

static void draw_wave(int16_t x0, int16_t y0, int16_t w, int16_t h)
{
    int16_t mid = (int16_t)(y0 + h / 2);
    int16_t amp = (int16_t)((h / 2) - 1);
    int16_t i, prev = mid;

    for (i = 0; i < w; i += 4) gfx_pixel((int16_t)(x0 + i), mid, C_WHITE);

    for (i = 0; i < w && i < WAVE_LEN; i++) {
        uint8_t k = (uint8_t)((ppg_wave_head + i) % WAVE_LEN);
        int16_t y = (int16_t)(mid - ((int32_t)ppg_wave[k] * amp) / 110);
        if (y < y0) y = y0;
        if (y > y0 + h - 1) y = (int16_t)(y0 + h - 1);
        if (i) gfx_line((int16_t)(x0 + i - 1), prev, (int16_t)(x0 + i), y, C_WHITE);
        else   gfx_pixel(x0, y, C_WHITE);
        prev = y;
    }
    /* leading-edge cursor so the trace reads as live */
    gfx_vline((int16_t)(x0 + w - 1), y0, h, C_WHITE);
}

/* value or dashes */
static void val_or_dash(int16_t x, int16_t y, uint16_t v_x10, uint8_t sc, uint8_t ok)
{
    if (ok) gfx_num_dec(x, y, v_x10, sc, C_WHITE);
    else    gfx_text_P(x, y, S_DASH3, sc, C_WHITE);
}

/* ======================= screen: monitor ======================= */
static void scr_monitor(void)
{
    title_bar(S_MONITOR);

    if (!ppg.finger && ppg.sps == 0) {
        /* No samples at all.  Read the config straight back off the chip:
         * if it matches what we wrote, the bus is fine and the sensor simply
         * is not converting -- which is a power problem, not a code one. */
        static uint32_t rb_ms;
        static uint8_t  mode, spo2, fifo, wr, ovf, rd, got;
        uint32_t nw = millis();
        if (!got || (uint32_t)(nw - rb_ms) >= 500) {
            got = 1; rb_ms = nw;
            max30102_readback(&mode, &spo2, &fifo);
            if (!max30102_ptrs(&wr, &ovf, &rd)) { wr = ovf = rd = 0xEE; }
        }
        gfx_text_P(8, ROW(0), PSTR("NO DATA FROM SENSOR"), 1, C_WHITE);
        /* MODE_CFG, SPO2_CFG and FIFO_CFG read back off the chip.  MODE and
         * SPO2 have single correct values; FIFO_CFG's top bits are the
         * averaging setting, so only the rollover bit is checked here. */
        gfx_text_P(2, ROW(1), PSTR("REG"), 1, C_WHITE);
        hex8(26, ROW(1), mode); hex8(56, ROW(1), spo2); hex8(86, ROW(1), fifo);
        gfx_text_P(2, ROW(2), PSTR("want  03    2F    50"), 1, C_WHITE);
        gfx_text_P(2, ROW(3), PSTR("WR"), 1, C_WHITE);  hex8(20, ROW(3), wr);
        gfx_text_P(52, ROW(3), PSTR("RD"), 1, C_WHITE); hex8(70, ROW(3), rd);
        /* Bus health.  TW/st is the last TWI failure, E the consecutive
         * error count, S the number of times the bus had to be forced
         * free -- the figure that exposed the STOP-after-NACK lockup. */
        gfx_text_P(2, ROW(4), PSTR("TW"), 1, C_WHITE);
        hex8(16, ROW(4), i2c_last_status());
        gfx_num(46, ROW(4), i2c_last_stage(), 1, C_WHITE);
        gfx_text_P(54, ROW(4), PSTR("E"), 1, C_WHITE);
        gfx_num(60, ROW(4), max30102_errors(), 1, C_WHITE);
        gfx_text_P(80, ROW(4), PSTR("S"), 1, C_WHITE);
        gfx_num(86, ROW(4), i2c_stuck_count(), 1, C_WHITE);
        return;
    }

    if (!ppg.finger) {
        gfx_heart(58, 18, 1, C_WHITE);
        gfx_text_P(13, 33, PSTR("ON THE SENSOR"), 1, C_WHITE);
        /* Live detector state.  If IR never climbs toward the threshold the
         * problem is optics or contact; if IR stays at 0 the sensor is not
         * streaming at all.  Either way it is visible without a debugger. */
        gfx_hline(0, 45, 128, C_WHITE);
        gfx_text_P(2, ROW(4), PSTR("IR"), 1, C_WHITE);
        gfx_num(16, ROW(4), (int32_t)ppg.refl_ir, 1, C_WHITE);
        gfx_text_P(58, ROW(4), PSTR("/"), 1, C_WHITE);
        gfx_num(64, ROW(4), (int32_t)ppg.finger_th, 1, C_WHITE);
        /* The learned idle level matters as much as the threshold: it is what
         * the threshold is derived from, so if it has drifted up to meet the
         * live reading the tracker -- not the optics -- is the problem. */
        gfx_text_P(2, ROW(5), PSTR("BL"), 1, C_WHITE);
        gfx_num(16, ROW(5), (int32_t)ppg.base_ir, 1, C_WHITE);
        gfx_num(58, ROW(5), ppg.sps, 1, C_WHITE);
        gfx_text_P(76, ROW(5), PSTR("/s"), 1, C_WHITE);
        gfx_num(94, ROW(5), (int32_t)((ppg.led_ir * 2 + 5) / 10), 1, C_WHITE);
        gfx_text_P(112, ROW(5), PSTR("mA"), 1, C_WHITE);
        return;
    }

    /* --- heart rate, large 7-segment --- */
    if (ppg.bpm_x10 >= 250)
        gfx_seg_num(2, 19, 16, 24, 4, 3, (uint16_t)(ppg.bpm_x10 / 10), 3, 1, C_WHITE);
    else
        gfx_text_P(8, 26, S_DASH3, 2, C_WHITE);
    gfx_text_P(20, 45, PSTR("BPM"), 1, C_WHITE);

    gfx_vline(60, 18, 28, C_WHITE);

    /* --- SpO2 --- */
    gfx_text_P(64, 18, PSTR("SpO2"), 1, C_WHITE);
    if (ppg.spo2_x10)
        gfx_seg_num(64, 27, 12, 17, 3, 2, (uint16_t)(ppg.spo2_x10 / 10), 3, 1, C_WHITE);
    else
        gfx_text_P(70, 30, S_DASH3, 2, C_WHITE);
    gfx_text_P(107, 36, PSTR("%"), 1, C_WHITE);

    /* --- perfusion index --- */
    gfx_text_P(64, 45, PSTR("PI"), 1, C_WHITE);
    if (ppg.pi_x100) {
        gfx_num_dec(78, 45, (int32_t)(ppg.pi_x100 / 10), 1, C_WHITE);
        gfx_text_P(102, 45, PSTR("%"), 1, C_WHITE);
    } else {
        gfx_text_P(78, 45, S_DASH3, 1, C_WHITE);
    }

    /* --- acquisition progress or live trace --- */
    if (!ppg.valid) {
        gfx_text_P(2, ROW(5), PSTR("ACQ"), 1, C_WHITE);
        gfx_progress(24, 54, 102, 9, ppg.progress);
    } else {
        draw_wave(0, 53, 128, 11);
    }
}

/* ======================= screen: waveform ======================= */
static void scr_wave(void)
{
    title_bar(S_WAVE);
    gfx_text_P(2, ROW(0), PSTR("HR"), 1, C_WHITE);
    val_or_dash(18, ROW(0), ppg.bpm_x10, 1, ppg.bpm_x10 >= 250);
    gfx_text_P(56, ROW(0), PSTR("SpO2"), 1, C_WHITE);
    val_or_dash(86, ROW(0), ppg.spo2_x10, 1, ppg.spo2_x10 != 0);
    gfx_text_P(116, ROW(0), PSTR("%"), 1, C_WHITE);
    gfx_hline(0, 26, 128, C_WHITE);
    draw_wave(0, 28, 128, 36);
}

/* ======================= screen: trends ======================= */
static void plot_trend(const uint8_t *buf, int16_t y0, int16_t h,
                       uint8_t lo_d, uint8_t hi_d, PGM_P label, uint16_t cur_x10)
{
    uint8_t i;
    int16_t prev = -1;
    int16_t lo = lo_d, hi = hi_d;
    int16_t mn = 255, mx = 0;
    uint8_t any = 0;

    for (i = 0; i < TREND_LEN; i++) {
        uint8_t v = buf[i];
        if (!v) continue;
        any = 1;
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    if (any) {                                  /* pad the auto range */
        if (mn > lo) lo = (int16_t)(mn - 2);
        if (mx < hi) hi = (int16_t)(mx + 2);
    }
    if (lo < 0)   lo = 0;
    if (hi <= lo) hi = (int16_t)(lo + 1);

    gfx_hline(0, y0, 128, C_WHITE);
    gfx_hline(0, (int16_t)(y0 + h - 1), 128, C_WHITE);

    for (i = 0; i < TREND_LEN; i++) {
        uint8_t k = (uint8_t)((tr_head + i) % TREND_LEN);
        int16_t v = buf[k];
        int16_t y;
        if (!v) { prev = -1; continue; }
        if (v < lo) v = lo;
        if (v > hi) v = hi;
        y = (int16_t)(y0 + h - 2 - ((int32_t)(v - lo) * (h - 3)) / (hi - lo));
        if (prev >= 0) gfx_line((int16_t)(i * 2 - 2), prev, (int16_t)(i * 2), y, C_WHITE);
        else           gfx_pixel((int16_t)(i * 2), y, C_WHITE);
        prev = y;
    }

    /* label plate on the left so it stays readable over the trace */
    gfx_fill(0, (int16_t)(y0 + 2), 46, 9, C_BLACK);
    gfx_text_P(1, (int16_t)(y0 + 3), label, 1, C_WHITE);
    if (cur_x10) gfx_num(26, (int16_t)(y0 + 3), cur_x10 / 10, 1, C_WHITE);
    else         gfx_text_P(26, (int16_t)(y0 + 3), S_DASH3, 1, C_WHITE);
}

static void scr_trend(void)
{
    title_bar(S_TREND);
    plot_trend(tr_bpm,  BODY_Y,      23, 50, 100, PSTR("HR"),   ppg.bpm_x10);
    plot_trend(tr_spo2, BODY_Y + 24, 23, 88, 100, PSTR("SPO2"), ppg.spo2_x10);
}

/* ======================= screen: analysis ======================= */
static void row_label(int16_t y, PGM_P s) { gfx_text_P(2, y, s, 1, C_WHITE); }

static void scr_stats(void)
{
    uint32_t sec = (millis() - ppg.sess_start_ms) / 1000UL;

    title_bar(S_STATS);

    row_label(ROW(0), PSTR("TIME"));
    gfx_num_pad(52, ROW(0), (sec / 60) % 100, 2, 1, C_WHITE);
    gfx_text_P(64, ROW(0), PSTR(":"), 1, C_WHITE);
    gfx_num_pad(70, ROW(0), sec % 60, 2, 1, C_WHITE);
    gfx_text_P(94, ROW(0), PSTR("B"), 1, C_WHITE);
    gfx_num(100, ROW(0), (int32_t)ppg.beats, 1, C_WHITE);

    row_label(ROW(1), PSTR("HR m/a/M"));
    gfx_num(52, ROW(1), ppg.bpm_min_x10 / 10, 1, C_WHITE);
    gfx_num(76, ROW(1), ppg.bpm_avg_x10 / 10, 1, C_WHITE);
    gfx_num(100, ROW(1), ppg.bpm_max_x10 / 10, 1, C_WHITE);

    row_label(ROW(2), PSTR("SPO2 min"));
    val_or_dash(64, ROW(2), ppg.spo2_min_x10, 1, ppg.spo2_min_x10 != 0);
    gfx_text_P(94, ROW(2), PSTR("%"), 1, C_WHITE);

    row_label(ROW(3), PSTR("SDNN"));
    gfx_num(52, ROW(3), ppg.sdnn_ms, 1, C_WHITE);
    gfx_text_P(76, ROW(3), PSTR("RMSSD"), 1, C_WHITE);
    gfx_num(112, ROW(3), ppg.rmssd_ms, 1, C_WHITE);

    row_label(ROW(4), PSTR("RESP"));
    if (ppg.resp_bpm) {
        gfx_num(52, ROW(4), ppg.resp_bpm, 1, C_WHITE);
        gfx_text_P(70, ROW(4), PSTR("/min"), 1, C_WHITE);
    } else {
        gfx_text_P(52, ROW(4), S_DASH3, 1, C_WHITE);
    }

    row_label(ROW(5), PSTR("TEMP"));
    if (ppg.temp_x10 > -900) {
        gfx_num_dec(52, ROW(5), ppg.temp_x10, 1, C_WHITE);
        gfx_char(82, ROW(5), 0x7F, 1, C_WHITE);      /* degree glyph */
        gfx_text_P(88, ROW(5), PSTR("C"), 1, C_WHITE);
    } else {
        gfx_text_P(52, ROW(5), S_DASH3, 1, C_WHITE);
    }
    gfx_text_P(100, ROW(5), PSTR("Q"), 1, C_WHITE);
    gfx_num(108, ROW(5), ppg.sqi, 1, C_WHITE);
}

/* ======================= screen: sensor ======================= */
static void hex8(int16_t x, int16_t y, uint8_t v)
{
    const char *h = "0123456789ABCDEF";
    gfx_text_P(x, y, PSTR("0x"), 1, C_WHITE);
    gfx_char((int16_t)(x + 12), y, h[v >> 4],   1, C_WHITE);
    gfx_char((int16_t)(x + 18), y, h[v & 0x0F], 1, C_WHITE);
}

static void scr_sensor(void)
{
    title_bar(S_SENSOR);

    gfx_text_P(2, ROW(0), PSTR("PART"), 1, C_WHITE);
    hex8(34, ROW(0), max30102_part_id());
    gfx_text_P(70, ROW(0), PSTR("REV"), 1, C_WHITE);
    hex8(94, ROW(0), max30102_rev_id());

    gfx_text_P(2, ROW(1), PSTR("Fs"), 1, C_WHITE);
    gfx_num(20, ROW(1), ppg.sps, 1, C_WHITE);
    gfx_text_P(44, ROW(1), PSTR("/s cal"), 1, C_WHITE);
    gfx_num_dec(82, ROW(1), ppg.fs_x100 / 10, 1, C_WHITE);
    gfx_text_P(112, ROW(1), PSTR("Hz"), 1, C_WHITE);

    gfx_text_P(2, ROW(2), PSTR("LED R"), 1, C_WHITE);
    hex8(34, ROW(2), ppg.led_red);
    gfx_text_P(70, ROW(2), PSTR("IR"), 1, C_WHITE);
    hex8(94, ROW(2), ppg.led_ir);

    gfx_text_P(2, ROW(3), PSTR("DC IR"), 1, C_WHITE);
    gfx_num(38, ROW(3), (int32_t)ppg.dc_ir, 1, C_WHITE);
    gfx_text_P(80, ROW(3), PSTR("AC"), 1, C_WHITE);
    gfx_num(98, ROW(3), ppg.ac_ir, 1, C_WHITE);

    gfx_text_P(2, ROW(4), PSTR("DC RD"), 1, C_WHITE);
    gfx_num(38, ROW(4), (int32_t)ppg.dc_red, 1, C_WHITE);
    gfx_text_P(80, ROW(4), PSTR("AC"), 1, C_WHITE);
    gfx_num(98, ROW(4), ppg.ac_red, 1, C_WHITE);

    gfx_text_P(2, ROW(5), PSTR("R"), 1, C_WHITE);
    /* R in Q12 -> three decimals without floating point */
    gfx_num(20, ROW(5), ppg.r_q12 >> 12, 1, C_WHITE);
    gfx_text_P(26, ROW(5), PSTR("."), 1, C_WHITE);
    gfx_num_pad(32, ROW(5), (uint32_t)(((uint32_t)(ppg.r_q12 & 0x0FFF) * 1000UL) >> 12), 3, 1, C_WHITE);
    gfx_text_P(66, ROW(5), PSTR("IBI"), 1, C_WHITE);
    gfx_num(90, ROW(5), ppg.ibi_ms, 1, C_WHITE);
}

/* ======================= controls / help ======================= */
static void scr_help(void)
{
    gfx_text_P(2, 1, PSTR("CONTROLS"), 1, C_WHITE);
    gfx_text_P(2, 9, PSTR(FW_NAME " " FW_VERSION), 1, C_WHITE);
    gfx_hline(0, HDR_H - 1, 128, C_WHITE);
    gfx_text_P(2, ROW(0), PSTR("1 press  next"), 1, C_WHITE);
    gfx_text_P(2, ROW(1), PSTR("2 press  back"), 1, C_WHITE);
    gfx_text_P(2, ROW(2), PSTR("hold     menu/OK"), 1, C_WHITE);
    gfx_hline(0, ROW(3) + 3, 128, C_WHITE);
    gfx_text_P(2, ROW(4), PSTR("ATmega32 + MAX30102"), 1, C_WHITE);
    gfx_text_P(2, ROW(5), PSTR("diag: UART0 38400 8N1"), 1, C_WHITE);
}

static void scr_sleepwait(void)
{
    gfx_text_P(2, 1, PSTR("SLEEPING"), 1, C_WHITE);
    gfx_text_P(2, 9, PSTR("no finger detected"), 1, C_WHITE);
    gfx_hline(0, HDR_H - 1, 128, C_WHITE);
    gfx_seg_digit(54, 20, 20, 26, 5, (int8_t)(s_countdown % 10), C_WHITE);
    gfx_text_P(7, ROW(5), PSTR("press to stay awake"), 1, C_WHITE);
}

/* ======================= settings menu ======================= */
static void menu_value(uint8_t idx, int16_t y, uint8_t inv)
{
    uint8_t c = (uint8_t)(inv ? C_BLACK : C_WHITE);
    int16_t x = 74;
    switch (idx) {
        case MI_LED:
            gfx_text_P(x, y, cfg.led_mode == LED_MODE_AUTO ? V_AUTO :
                             cfg.led_mode == LED_MODE_LOW  ? V_LOW  :
                             cfg.led_mode == LED_MODE_MED  ? V_MED  : V_HIGH, 1, c);
            break;
        case MI_AVG:
            gfx_num(x, y, 1 << cfg.avg_code, 1, c);
            gfx_text_P((int16_t)(x + gfx_num_w(1 << cfg.avg_code, 1)), y, PSTR("x"), 1, c);
            break;
        case MI_CAL:
            if (cfg.spo2_cal >= 0) gfx_text_P(x, y, PSTR("+"), 1, c);
            gfx_num_dec((int16_t)(x + 6), y, cfg.spo2_cal, 1, c);
            break;
        case MI_CONTRAST: gfx_num(x, y, cfg.contrast, 1, c); break;
        case MI_FLIP:     gfx_text_P(x, y, cfg.flip ? V_ON : V_OFF, 1, c); break;
        case MI_BEEP:     gfx_text_P(x, y, cfg.beep ? V_ON : V_OFF, 1, c); break;
        case MI_DIM:
            if (!cfg.dim_s) gfx_text_P(x, y, V_OFF, 1, c);
            else { gfx_num(x, y, cfg.dim_s, 1, c); gfx_text_P((int16_t)(x + 12), y, PSTR("s"), 1, c); }
            break;
        case MI_SLEEP:
            if (!cfg.sleep_s) gfx_text_P(x, y, V_OFF, 1, c);
            else if (cfg.sleep_s < 60) {
                gfx_num(x, y, cfg.sleep_s, 1, c);
                gfx_text_P((int16_t)(x + 12), y, PSTR("s"), 1, c);
            } else {
                gfx_num(x, y, cfg.sleep_s / 60, 1, c);
                gfx_text_P((int16_t)(x + 6), y, PSTR("min"), 1, c);
            }
            break;
        case MI_START:
            gfx_text_P(x, y, (PGM_P)pgm_read_word(&scr_names[cfg.start_scr % SCR_COUNT]), 1, c);
            break;
        default:
            gfx_text_P(x, y, V_GO, 1, c);
            break;
    }
}

#define MENU_ROWS 4
#define MROW_H   12            /* four items inside the 47-row body */
#define MROW_Y0  18

static void scr_menu(void)
{
    uint8_t i;
    gfx_text_P(2, 1, S_MENU, 1, C_WHITE);
    gfx_num(104, 1, (int32_t)(s_sel + 1), 1, C_WHITE);
    gfx_text_P(112, 1, PSTR("/"), 1, C_WHITE);
    gfx_num(118, 1, MI_COUNT, 1, C_WHITE);
    gfx_text_P(2, 9, PSTR("1x next  hold OK  2x back"), 1, C_WHITE);
    gfx_hline(0, HDR_H - 1, 128, C_WHITE);

    for (i = 0; i < MENU_ROWS; i++) {
        uint8_t idx = (uint8_t)(s_top + i);
        int16_t y   = (int16_t)(MROW_Y0 + i * MROW_H);
        uint8_t inv = (uint8_t)(idx == s_sel);
        if (idx >= MI_COUNT) break;
        if (inv) gfx_fill(0, (int16_t)(y - 2), 126, MROW_H - 1, C_WHITE);
        gfx_text_P(3, y, (PGM_P)pgm_read_word(&menu_names[idx]), 1,
                   (uint8_t)(inv ? C_BLACK : C_WHITE));
        menu_value(idx, y, inv);
    }
    /* scrollbar */
    gfx_vline(126, BODY_Y - 1, 48, C_WHITE);
    {
        int16_t h = (int16_t)(48 * MENU_ROWS / MI_COUNT);
        int16_t y = (int16_t)(BODY_Y - 1 + (int32_t)48 * s_top / MI_COUNT);
        if (h < 4) h = 4;
        gfx_fill(125, y, 3, h, C_WHITE);
    }
}

static PGM_P edit_enum_str(void)
{
    switch (s_sel) {
        case MI_LED:   return cfg.led_mode == LED_MODE_AUTO ? V_AUTO :
                              cfg.led_mode == LED_MODE_LOW  ? V_LOW  :
                              cfg.led_mode == LED_MODE_MED  ? V_MED  : V_HIGH;
        case MI_FLIP:  return cfg.flip ? V_ON : V_OFF;
        case MI_BEEP:  return cfg.beep ? V_ON : V_OFF;
        case MI_START: return (PGM_P)pgm_read_word(&scr_names[cfg.start_scr % SCR_COUNT]);
        default:       return V_OFF;
    }
}

static void scr_edit(void)
{
    gfx_text_P(2, 1, (PGM_P)pgm_read_word(&menu_names[s_sel]), 1, C_WHITE);
    gfx_text_P(2, 9, PSTR("1x change  hold save"), 1, C_WHITE);
    gfx_hline(0, HDR_H - 1, 128, C_WHITE);
    gfx_rrect(10, 20, 108, 26, C_WHITE);

    switch (s_sel) {
        case MI_CONTRAST:
            gfx_num((int16_t)(64 - gfx_num_w(cfg.contrast, 2) / 2), 27,
                    cfg.contrast, 2, C_WHITE);
            break;
        case MI_AVG:
            gfx_num(52, 27, 1 << cfg.avg_code, 2, C_WHITE);
            gfx_text_P((int16_t)(52 + gfx_num_w(1 << cfg.avg_code, 2)), 27,
                       PSTR("x"), 2, C_WHITE);
            break;
        case MI_CAL:
            gfx_text_P(34, 27, cfg.spo2_cal >= 0 ? PSTR("+") : PSTR("-"), 2, C_WHITE);
            gfx_num_dec(46, 27, (int32_t)(cfg.spo2_cal < 0 ? -cfg.spo2_cal
                                                           : cfg.spo2_cal), 2, C_WHITE);
            gfx_text_P(94, 27, PSTR("%"), 2, C_WHITE);
            break;
        case MI_DIM:
            if (!cfg.dim_s) {
                gfx_text_P((int16_t)(64 - gfx_text_w_P(V_OFF, 2) / 2), 27, V_OFF, 2, C_WHITE);
            } else {
                gfx_num(52, 27, cfg.dim_s, 2, C_WHITE);
                gfx_text_P((int16_t)(52 + gfx_num_w(cfg.dim_s, 2)), 27, PSTR("s"), 2, C_WHITE);
            }
            break;
        case MI_SLEEP:
            if (!cfg.sleep_s) {
                gfx_text_P((int16_t)(64 - gfx_text_w_P(V_OFF, 2) / 2), 27, V_OFF, 2, C_WHITE);
            } else if (cfg.sleep_s < 60) {
                gfx_num(46, 27, cfg.sleep_s, 2, C_WHITE);
                gfx_text_P(70, 27, PSTR("s"), 2, C_WHITE);
            } else {
                gfx_num(40, 27, cfg.sleep_s / 60, 2, C_WHITE);
                gfx_text_P(52, 27, PSTR("min"), 2, C_WHITE);
            }
            break;
        default: {
            PGM_P v = edit_enum_str();
            gfx_text_P((int16_t)(64 - gfx_text_w_P(v, 2) / 2), 27, v, 2, C_WHITE);
            break;
        }
    }

    gfx_text_P(4, ROW(4), PSTR("1x  change value"), 1, C_WHITE);
    gfx_text_P(4, ROW(5), PSTR("hold OK   2x cancel"), 1, C_WHITE);
}

/* ======================= interaction ======================= */
static void menu_scroll_fix(void)
{
    if (s_sel < s_top) s_top = s_sel;
    if (s_sel >= (uint8_t)(s_top + MENU_ROWS)) s_top = (uint8_t)(s_sel - MENU_ROWS + 1);
    if (s_top + MENU_ROWS > MI_COUNT)
        s_top = (uint8_t)(MI_COUNT > MENU_ROWS ? MI_COUNT - MENU_ROWS : 0);
}

static void edit_next(void)
{
    switch (s_sel) {
        case MI_LED:      cfg.led_mode = (uint8_t)((cfg.led_mode + 1) & 3); break;
        case MI_AVG:      cfg.avg_code = (uint8_t)((cfg.avg_code + 1) % 6);
                          max30102_set_avg(cfg.avg_code);
                          ppg_reset_measure(); break;
        case MI_CAL:      cfg.spo2_cal = (int8_t)(cfg.spo2_cal + 5);
                          if (cfg.spo2_cal > 50) cfg.spo2_cal = -50;
                          break;
        case MI_CONTRAST: cfg.contrast = (uint8_t)(cfg.contrast + 32);
                          ssd1306_contrast(cfg.contrast); break;
        case MI_FLIP:     cfg.flip = (uint8_t)!cfg.flip;
                          ssd1306_flip(cfg.flip); break;
        case MI_BEEP:     cfg.beep = (uint8_t)!cfg.beep;
                          if (cfg.beep) buz_beep(2400, 40);
                          break;
        case MI_DIM:      cfg.dim_s = (uint8_t)(cfg.dim_s == 0  ? 15 :
                                                cfg.dim_s == 15 ? 30 :
                                                cfg.dim_s == 30 ? 60 : 0); break;
        case MI_SLEEP:    cfg.sleep_s = (uint16_t)(cfg.sleep_s == 0   ?  30 :
                                                   cfg.sleep_s == 30  ?  60 :
                                                   cfg.sleep_s == 60  ? 120 :
                                                   cfg.sleep_s == 120 ? 300 : 0);
                          break;
        case MI_START:    cfg.start_scr = (uint8_t)((cfg.start_scr + 1) % SCR_COUNT); break;
        default: break;
    }
}

static uint8_t item_is_action(uint8_t i)
{
    return (uint8_t)(i == MI_RESET_SESSION || i == MI_FACTORY ||
                     i == MI_HELP || i == MI_SAVE);
}

static void run_action(uint8_t i)
{
    switch (i) {
        case MI_RESET_SESSION:
            ppg_reset_session();
            memset(tr_bpm, 0, sizeof(tr_bpm));
            memset(tr_spo2, 0, sizeof(tr_spo2));
            toast(S_CLEARED);
            break;
        case MI_FACTORY:
            settings_defaults();
            settings_apply();
            toast(S_DEFAULTS);
            break;
        case MI_HELP:
            s_mode = MODE_HELP;
            break;
        case MI_SAVE:
            settings_save();
            settings_apply();
            toast(S_SAVED);
            s_mode = MODE_SCREEN;
            break;
        default: break;
    }
}

void ui_event(btn_evt_t e)
{
    if (e == BTN_NONE) return;
    s_last_input_ms = s_idle_ms = millis();

    if (s_mode == MODE_SLEEPWAIT) {      /* a press means "stay awake" */
        s_mode = s_mode_saved;
        return;
    }
    if (s_dimmed) {                      /* first press only wakes the panel */
        s_dimmed = 0;
        ssd1306_contrast(cfg.contrast);
        return;
    }
    buz_beep(3000, 12);

    switch (s_mode) {
        case MODE_SCREEN:
            if (e == BTN_CLICK)       s_scr = (uint8_t)((s_scr + 1) % SCR_COUNT);
            else if (e == BTN_DOUBLE) s_scr = (uint8_t)((s_scr + SCR_COUNT - 1) % SCR_COUNT);
            else {
                s_backup = cfg;
                s_mode = MODE_MENU;
                s_sel = s_top = 0;
            }
            break;

        case MODE_MENU:
            if (e == BTN_CLICK) {
                s_sel = (uint8_t)((s_sel + 1) % MI_COUNT);
                menu_scroll_fix();
            } else if (e == BTN_DOUBLE) {
                s_mode = MODE_SCREEN;
            } else {
                if (item_is_action(s_sel)) run_action(s_sel);
                else                        s_mode = MODE_EDIT;
            }
            break;

        case MODE_EDIT:
            if (e == BTN_CLICK) {
                edit_next();
            } else if (e == BTN_DOUBLE) {         /* cancel: restore the field */
                switch (s_sel) {
                    case MI_LED:      cfg.led_mode = s_backup.led_mode; break;
                    case MI_AVG:      cfg.avg_code = s_backup.avg_code;
                                      max30102_set_avg(cfg.avg_code); break;
                    case MI_CAL:      cfg.spo2_cal = s_backup.spo2_cal; break;
                    case MI_CONTRAST: cfg.contrast = s_backup.contrast;
                                      ssd1306_contrast(cfg.contrast); break;
                    case MI_FLIP:     cfg.flip = s_backup.flip;
                                      ssd1306_flip(cfg.flip); break;
                    case MI_BEEP:     cfg.beep = s_backup.beep; break;
                    case MI_DIM:      cfg.dim_s = s_backup.dim_s; break;
                    case MI_SLEEP:    cfg.sleep_s = s_backup.sleep_s; break;
                    case MI_START:    cfg.start_scr = s_backup.start_scr; break;
                    default: break;
                }
                s_mode = MODE_MENU;
            } else {
                s_backup = cfg;
                settings_save();
                toast(S_SAVED);
                s_mode = MODE_MENU;
            }
            break;

        case MODE_HELP:
        default:
            s_mode = MODE_SCREEN;
            break;
    }
}

/* ======================= periodic housekeeping ======================= */
void ui_tick(void)
{
    uint32_t now = millis();

    if (ppg.finger) s_idle_ms = now;

    /* --- idle countdown into deep sleep --- */
    if (cfg.sleep_s) {
        uint32_t idle_s = (uint32_t)(now - s_idle_ms) / 1000UL;
        if (idle_s >= cfg.sleep_s) {
            s_sleep_req = 1;
        } else if (idle_s + SLEEP_COUNTDOWN_S >= cfg.sleep_s) {
            if (s_mode != MODE_SLEEPWAIT) { s_mode_saved = s_mode; s_mode = MODE_SLEEPWAIT; }
            s_countdown = (uint8_t)(cfg.sleep_s - idle_s);
        } else if (s_mode == MODE_SLEEPWAIT) {
            s_mode = s_mode_saved;
        }
    } else if (s_mode == MODE_SLEEPWAIT) {
        s_mode = s_mode_saved;
    }

    /* --- status LED --- */
    if      (s_mode == MODE_SLEEPWAIT) led_mode(LED_COUNTDOWN);
    else if (!ppg.finger)              led_mode(LED_BREATHE);
    else if (!ppg.valid)               led_mode(LED_SEARCH);
    else                               led_mode(LED_BEAT);

    if ((uint32_t)(now - tr_ms) >= 2000) {
        tr_ms = now;
        tr_bpm[tr_head]  = (uint8_t)(ppg.valid && ppg.bpm_x10 >= 250
                                     ? (ppg.bpm_x10 / 10 > 250 ? 250 : ppg.bpm_x10 / 10) : 0);
        tr_spo2[tr_head] = (uint8_t)(ppg.valid && ppg.spo2_x10 ? ppg.spo2_x10 / 10 : 0);
        tr_head = (uint8_t)((tr_head + 1) % TREND_LEN);
    }

    if (cfg.dim_s && !s_dimmed &&
        (uint32_t)(now - s_last_input_ms) > (uint32_t)cfg.dim_s * 1000UL) {
        s_dimmed = 1;
        ssd1306_contrast(0x01);
    }

    if (ppg.beat) {
        ppg.beat = 0;
        led_flash();
        if (cfg.beep) buz_beep(2600, 25);
    }
}

/* ======================= frame ======================= */
void ui_draw(void)
{
    gfx_clear();

    switch (s_mode) {
        case MODE_MENU: scr_menu(); break;
        case MODE_EDIT: scr_edit(); break;
        case MODE_HELP: scr_help(); break;
        case MODE_SLEEPWAIT: scr_sleepwait(); break;
        default:
            switch (s_scr) {
                case SCR_WAVE:   scr_wave();   break;
                case SCR_TREND:  scr_trend();  break;
                case SCR_STATS:  scr_stats();  break;
                case SCR_SENSOR: scr_sensor(); break;
                default:         scr_monitor();break;
            }
            break;
    }

    if (s_toast && (int32_t)(millis() - s_toast_ms) < 0) {
        uint8_t w = (uint8_t)(gfx_text_w_P(s_toast, 1) + 8);
        int16_t x = (int16_t)(64 - w / 2);
        gfx_fill(x, 26, w, 13, C_BLACK);
        gfx_rrect(x, 26, w, 13, C_WHITE);
        gfx_text_P((int16_t)(x + 4), 29, s_toast, 1, C_WHITE);
    } else {
        s_toast = 0;
    }
}
