#include "ppg.h"
#include "sys.h"
#include "max30102.h"
#include "settings.h"
#include "dbg.h"
#include <string.h>

ppg_state_t ppg;

int8_t  ppg_wave[WAVE_LEN];
uint8_t ppg_wave_head;

/* ------------------------------------------------------------------
 * Signal chain (all fixed point, O(1) memory):
 *
 *   raw --> DC tracker (1-pole HP, 0.25 Hz) --> HP2 (1-pole, 0.5 Hz)
 *       --> LP x2 (5 Hz)  --> band-limited PPG  f
 *
 * Both channels get the identical filter, so the red/IR amplitude ratio
 * that SpO2 depends on is preserved exactly while baseline wander from
 * breathing and finger movement is removed.
 *
 * Beats are taken from the UPWARD ZERO CROSSING of the band-passed IR
 * trace, as the reference implementation does.  A band-passed PPG crosses
 * zero upward exactly once per cardiac cycle regardless of pulse height,
 * so the detector does not care about amplitude at all -- amplitude is used
 * only to reject noise.  The peak/trough envelope is still tracked, but now
 * only to normalise the waveform for the display.
 * ------------------------------------------------------------------ */
#define DC_SHIFT        6       /* DC tracker, ~0.25 Hz corner              */
#define HP2_SHIFT       5       /* second high-pass, ~0.5 Hz corner         */
#define LP_SHIFT        3       /* two cascaded 1-pole LPs, ~2.4 Hz corner  */
#define ENV_DECAY       8       /* peak/trough envelope decay, ~2.6 s       */
#define AMP_FLOOR      60       /* display normalisation floor              */
#define AMP_MIN_BEAT   25       /* below this the trace is treated as noise */
#define LOST_RESYNC     4       /* lose this many samples -> drop the beat   */
#define IBI_MIN_MS    250       /* 240 bpm ceiling (reference: 255)         */
#define IBI_MAX_MS   3000       /* 20 bpm floor  (reference: 20)           */
#define REFRAC_MS     200       /* hold-off floor, 300 bpm ceiling          */
#define REFRAC_MAX    700       /* never blank longer than this             */
#define IBI_HIST       12
#define R_HIST          8
#define SETTLE_SAMPLES 40       /* samples discarded after an AGC step      */
#define LOCK_BEATS      4       /* beats before a reading is published      */

/* Largest R the SpO2 curve is still calibrated for, in Q8.  295/256 = 1.152,
 * which is where the polynomial passes 70 % -- the bottom of Maxim's table
 * and the bottom of anything this optical front end can be trusted for. */
#define R_TRUST_MAX   295

/* --- AC amplitude and channel agreement, both taken from the reference ---
 *
 * Maxim's own algorithm (algorithm_by_RF.cpp, the RD117 successor) forms R
 * from the RMS of each band-passed channel, not from its peak-to-peak span:
 *
 *     f_y_ac = rf_rms(an_y, n, &f_red_sumsq);      // RED
 *     f_x_ac = rf_rms(an_x, n, &f_ir_sumsq);       // IR
 *     xy_ratio = (f_y_ac*f_ir_mean)/(f_x_ac*f_red_mean);
 *
 * We do NOT follow it there, and the reason is measured rather than assumed:
 * see the note in spo2_update().  An RMS taken from integer accumulators
 * over a single beat loses more to quantisation on the weak RED channel than
 * peak-to-peak loses to noise bias.
 *
 * The reference also gates on the Pearson correlation between the two
 * band-passed channels and refuses to report at all below 0.8:
 *
 *     if(*correl >= min_pearson_correlation) { ... }
 *     const float min_pearson_correlation = 0.8;
 *
 * A cardiac pulse drives RED and IR in lockstep, so a genuine measurement
 * correlates at better than 0.95.  Ambient light, movement, optical
 * crosstalk or a channel that is not seeing its emitter all break that
 * lockstep while still leaving a plausible-looking R.  Correlation is
 * therefore the test that separates "R is telling me about blood" from
 * "R is telling me about something else", and it costs three running sums.
 *
 * Both need only Sx2, Sy2 and Sxy, so the correlation is free once the RMS
 * is being accumulated:  r = Sxy / sqrt(Sx2 * Sy2).
 */
#define RMS_SHIFT       3       /* keeps the sums of squares inside a uint32 */
#define RMS_Q_MAX    2047       /* per-sample clamp in the shifted domain    */
#define RMS_N_MAX     512       /* and a cap on the window, for the same     */
#define CORR_MIN_N      4       /* 0.8 as a fraction: correlation >= 4/5     */
#define CORR_MIN_D      5

/* Respiration (RIIV): the breathing cycle rides on the IR baseline.  A
 * 2-pole low-pass at 0.25 Hz removes the cardiac component, a slow
 * high-pass removes drift, and crossings of the result are counted over
 * a 30 s window.  Costs three accumulators instead of a sample buffer. */
#define RESP_LP_SHIFT   6
#define RESP_HP_SHIFT   9
#define RESP_ENV_SHIFT  9
#define RESP_WIN     3000       /* samples (30 s at 100 Hz)                 */
#define RESP_FLOOR_DIV 4000     /* modulation below DC/4000 is not breathing*/

/* Finger detection.
 *
 * The criterion is a genuine rise of the IR DC level over the *learned* idle
 * reflection, because no single absolute count is right for every module: how
 * much light comes back with nothing on the sensor varies by an order of
 * magnitude between boards, depending on how well the plastic isolates the
 * emitters from the photodiode.
 *
 * Three bounds shape that rise, and getting their direction right is the
 * whole game:
 *
 *   FINGER_FLOOR_IR  a lower bound, so ambient IR wobble on a module with a
 *                    very dark idle level cannot trigger detection.
 *
 *   FINGER_CAP_IR    an UPPER bound, taken from the reference implementation
 *                    (SparkFun / SunFounder lesson 14 call it a finger past
 *                    50000 counts at LED_PA_REF with the ADC at 4096 nA).  A
 *                    threshold above that is past what a real finger returns,
 *                    so it means the baseline tracker has been poisoned -- it
 *                    latched a level measured with a finger already resting on
 *                    the sensor, or at a different LED current.  Clamping lets
 *                    the tracker recover instead of locking detection out.
 *
 *                    This is a cap, NOT a floor.  Requiring 50000 outright
 *                    fails every module that simply does not reflect that
 *                    brightly, and requiring 3x the learned baseline (the
 *                    original rule) put the threshold at 123000 on a module
 *                    idling at 40000 -- higher than any finger reads.  Both
 *                    directions of that mistake produce the same symptom: a
 *                    sensor that streams perfectly and never sees a finger.
 *
 *   FINGER_MIN_RISE  the rise itself, ADDITIVE and proportional, so it scales
 *                    gently with the idle level instead of multiplying it.
 */
#define FINGER_MIN_RISE  4000UL   /* absolute part of the required rise   */
#define FINGER_FLOOR_IR  6000UL   /* never trigger below this             */
#define FINGER_CAP_IR   50000UL   /* reference count, at LED_PA_REF       */
#define FINGER_NOBEAT_MS 15000UL  /* latched but pulseless -> let go        */
#define LED_PA_SEARCH  LED_PA_REF /* fixed drive while hunting for a finger */

typedef struct {
    /* --- filter state --- */
    int32_t  dcacc_ir, dcacc_red;
    int32_t  hp_ir, hp_red;
    int32_t  lp1, lp2;              /* IR  */
    int32_t  lp1r, lp2r;            /* RED */
    int32_t  fprev;
    int32_t  pk, tr;                /* dual envelope */
    int32_t  amp;                   /* half the envelope span */
    uint8_t  dc_init;

    /* --- beat detector --- */
    uint32_t nsamp;
    uint32_t t_cross_q8;
    int32_t  cyc_max, cyc_min;      /* peak and trough of the current cycle */
    int32_t  amp_avg;               /* running mean of accepted amplitudes  */
    uint8_t  settle;
    uint16_t refrac;                /* samples left in the beat refractory */
    uint8_t  half;                  /* waveform decimator */

    /* --- per-beat measurement windows --- */
    int32_t  fir_min, fir_max;      /* filtered IR  -> R  */
    int32_t  fred_min, fred_max;    /* filtered RED -> R  */
    uint32_t rir_min, rir_max;      /* raw IR       -> PI          */
    uint32_t rred_min, rred_max;    /* raw RED      -> diagnostics */
    uint8_t  win_valid;
    /* RMS and cross-product accumulators for the window above.  Sx2/Sy2 give
     * each channel's AC; Sxy with them gives the red/IR correlation. */
    uint32_t ssq_ir, ssq_red;
    int32_t  sxy;
    uint16_t ssq_n;

    /* --- history rings --- */
    uint16_t ibi[IBI_HIST];
    uint8_t  ibi_n, ibi_i;
    uint16_t rq12[R_HIST];
    uint8_t  r_n, r_i;

    /* --- respiration --- */
    int32_t  r_lp1, r_lp2, r_hp, r_env;
    uint8_t  r_init, r_above;
    uint16_t r_cross, r_win, r_low;

    uint32_t base_dc;               /* learned no-finger reflection level */
    uint16_t base_cnt;
    uint8_t  finger_cnt;
    uint8_t  no_arm;                /* block re-latching until DC falls back */
    uint32_t agc_ms, temp_ms, fs_ms, sps_ms;
    uint32_t fs_t0;                 /* origin of the sample-rate baseline  */
    uint8_t  fs_saved;              /* rate already written to EEPROM      */
    uint32_t fs_nmark, sps_mark;
} ppg_priv_t;

static ppg_priv_t s;

/* ------------------------------------------------------------------ */
static uint8_t led_fixed_pa(void)
{
    switch (cfg.led_mode) {
        case LED_MODE_LOW:  return 0x14;
        case LED_MODE_MED:  return 0x28;
        case LED_MODE_HIGH: return LED_PA_MAX;  /* the ceiling, whatever it is */
        default:            return LED_PA_SEARCH;   /* AUTO starts at the ref */
    }
}

void ppg_reset_measure(void)
{
    s.dc_init = 0;
    s.hp_ir = s.hp_red = 0;
    s.lp1 = s.lp2 = s.lp1r = s.lp2r = s.fprev = 0;
    s.pk = s.tr = 0;
    s.amp = 0;
    s.refrac = 0;
    s.cyc_max = s.cyc_min = 0;
    s.amp_avg = 0;
    s.win_valid = 0;
    s.ibi_n = s.ibi_i = 0;
    s.r_n = s.r_i = 0;
    s.t_cross_q8 = 0;
    s.r_init = 0;
    s.r_cross = s.r_win = s.r_low = 0;
    s.r_above = 0;
    s.r_env = 0;
    s.base_cnt = 0;

    ppg.valid     = 0;
    ppg.progress  = 0;
    ppg.bpm_x10   = 0;
    ppg.spo2_x10  = 0;
    ppg.pi_x100   = 0;
    ppg.ibi_ms    = 0;
    ppg.sdnn_ms   = 0;
    ppg.rmssd_ms  = 0;
    ppg.resp_bpm  = 0;
    ppg.sqi       = 0;
    ppg.wave      = 0;
    ppg.spo2_rail = 0;
    ppg.corr_x100 = 0;
    ppg.fac_ir    = 0;
    ppg.fac_red   = 0;
    ppg.r_q12     = 0;
    memset(ppg_wave, 0, sizeof(ppg_wave));
}

void ppg_reset_session(void)
{
    ppg.beats        = 0;
    ppg.rejects      = 0;
    ppg.bpm_min_x10  = 0;
    ppg.bpm_max_x10  = 0;
    ppg.bpm_avg_x10  = 0;
    ppg.spo2_min_x10 = 0;
    ppg.sess_start_ms = millis();
}

void ppg_init(void)
{
    memset(&s, 0, sizeof(s));
    memset(&ppg, 0, sizeof(ppg));
    /* Start from the rate this module was measured at last time rather than
     * the nominal 100 Hz.  The MAX30102 oscillator is only good to a few
     * percent and this board runs about 120 Hz where 100 is nominal -- a
     * 20 % error straight onto the BPM figure for the twenty-odd seconds the
     * live estimator needs to converge, which is most of a spot measurement. */
    ppg.fs_x100 = cfg.fs_cal ? cfg.fs_cal : (uint16_t)(PPG_FS_NOM * 100u);
    ppg.led_ir  = led_fixed_pa();
    ppg.led_red = ppg.led_ir;
    ppg.temp_x10 = -999;
    max30102_set_leds(ppg.led_red, ppg.led_ir);
    ppg_reset_measure();
    ppg_reset_session();
    s.agc_ms = s.temp_ms = s.fs_ms = s.sps_ms = s.fs_t0 = millis();
}

/* ---------------- HRV over the accepted-interval ring ---------------- */
static void hrv_update(void)
{
    uint8_t n = s.ibi_n, i;
    uint32_t sum = 0, var = 0, sd = 0;
    uint16_t mean;
    if (n < 4) { ppg.sdnn_ms = 0; ppg.rmssd_ms = 0; return; }
    for (i = 0; i < n; i++) sum += s.ibi[i];
    mean = (uint16_t)(sum / n);
    for (i = 0; i < n; i++) {
        int32_t d = (int32_t)s.ibi[i] - mean;
        var += (uint32_t)(d * d);
    }
    ppg.sdnn_ms = isqrt32(var / n);

    for (i = 1; i < n; i++) {
        uint8_t a = (uint8_t)((s.ibi_i + IBI_HIST - i)     % IBI_HIST);
        uint8_t b = (uint8_t)((s.ibi_i + IBI_HIST - i - 1) % IBI_HIST);
        int32_t d = (int32_t)s.ibi[a] - (int32_t)s.ibi[b];
        sd += (uint32_t)(d * d);
    }
    ppg.rmssd_ms = isqrt32(sd / (n - 1));
}

static void sqi_update(void)
{
    int16_t q = 100;
    if (s.ibi_n < LOCK_BEATS) q = (int16_t)(s.ibi_n * (100 / LOCK_BEATS));
    if (ppg.pi_x100 < 20) q -= (int16_t)((20 - ppg.pi_x100) * 3);
    if (ppg.ibi_ms && ppg.sdnn_ms) {
        int16_t rel = (int16_t)(((uint32_t)ppg.sdnn_ms * 100u) / ppg.ibi_ms);
        if (rel > 6) q -= (int16_t)((rel - 6) * 2);
    }
    if (ppg.beats + ppg.rejects > 8) {
        uint16_t rr = (uint16_t)((ppg.rejects * 100u) / (ppg.beats + ppg.rejects));
        q -= (int16_t)rr;
    }
    if (q < 0)   q = 0;
    if (q > 100) q = 100;
    ppg.sqi = (uint8_t)q;
}

/* ---------------- SpO2 for one completed beat window ---------------- */
static void spo2_update(void)
{
    uint32_t aci, acr, raci, racr, ri, rr, rq16;
    if (!s.win_valid) return;

    aci  = (uint32_t)(s.fir_max  - s.fir_min);      /* filtered, drives R    */
    acr  = (uint32_t)(s.fred_max - s.fred_min);
    raci = s.rir_max  - s.rir_min;                  /* raw, drives PI        */
    racr = s.rred_max - s.rred_min;                 /* raw, diagnostics only */

    if (aci < 20 || acr < 5) return;
    if (ppg.dc_ir < 2000 || ppg.dc_red < 2000) return;
    if (aci  > 60000UL) aci  = 60000UL;
    if (acr  > 60000UL) acr  = 60000UL;
    if (raci > 60000UL) raci = 60000UL;
    if (racr > 60000UL) racr = 60000UL;

    ppg.ac_ir   = (uint16_t)raci;
    ppg.ac_red  = (uint16_t)racr;
    ppg.pi_x100 = (uint16_t)((raci * 10000UL) / ppg.dc_ir);

    /* --- red/IR agreement gate, from the reference --- */
    if (s.ssq_n < 16) return;                       /* too short to judge */
    {
        uint32_t rq_i = isqrt32(s.ssq_ir  / s.ssq_n);   /* RMS, shifted domain */
        uint32_t rq_r = isqrt32(s.ssq_red / s.ssq_n);
        int32_t  cx   = s.sxy / (int32_t)s.ssq_n;       /* mean cross product  */
        uint32_t den;

        if (rq_i == 0 || rq_r == 0) return;
        den = rq_i * rq_r;

        /* Pearson r = Sxy / sqrt(Sx2*Sy2) = cx / (rms_i*rms_r).  Compared as
         * a fraction so no square root or float is needed, and so everything
         * stays inside a uint32: cx and den are both at most RMS_Q_MAX^2 =
         * 4.2e6, and five times that still fits.
         *
         * Only the RATIO of these sums is used, never their absolute size,
         * which is why the coarse RMS_SHIFT quantisation is harmless here --
         * it scales numerator and denominator alike. */
        ppg.corr_x100 = (uint8_t)((cx <= 0) ? 0
                        : ((uint32_t)cx >= den) ? 100
                        : (uint8_t)(((uint32_t)cx * 100UL) / den));
        if (cx <= 0 || (uint32_t)cx * CORR_MIN_D < den * CORR_MIN_N) {
            /* RED and IR are not moving together, so whatever R comes out of
             * them is not a measurement of blood.  The reference returns
             * -999 here; we publish nothing and say why. */
            ppg.spo2_rail = 2;
            ppg.spo2_x10  = 0;
            return;
        }
    }
    /* The AC that forms R stays peak-to-peak, NOT the reference's RMS.
     * The reference works in float over a four-second window, where the
     * choice costs nothing; here the window is one beat and the arithmetic
     * is integer, and an RMS taken from these shifted accumulators was
     * measured at up to 7 % error on R against under 1.6 % for the span --
     * quantisation of the weaker RED channel, which is exactly the channel
     * that must not be degraded.  RMS was adopted upstream to stop noise
     * inflating the span; the correlation gate above rejects those signals
     * outright, which addresses the same problem without the precision
     * cost. */
    ppg.fac_ir  = (uint16_t)aci;
    ppg.fac_red = (uint16_t)acr;

    ri = (aci << 16) / ppg.dc_ir;                   /* AC/DC in Q16 */
    rr = (acr << 16) / ppg.dc_red;
    if (ri < 1) ri = 1;
    if (ri > 32767UL) ri = 32767UL;
    if (rr > 32767UL) rr = 32767UL;

    rq16 = (rr << 16) / ri;                         /* ratio of ratios */
    if (rq16 > 0xFFFFUL * 4) rq16 = 0xFFFFUL * 4;

    s.rq12[s.r_i] = (uint16_t)(rq16 >> 4);
    s.r_i = (uint8_t)((s.r_i + 1) % R_HIST);
    if (s.r_n < R_HIST) s.r_n++;

    {
        uint16_t med = median_u16(s.rq12, s.r_n);
        int32_t  r8  = med >> 4;                    /* R in Q8 */
        int32_t  r2, sp;
        ppg.r_q12 = med;

        /* The curve below is a fit to Maxim's reference table and is only
         * meaningful over the range that table covers.  Past R = 1.152 it
         * passes 70 % and dives -- it reaches zero at R = 1.9 and is negative
         * beyond -- so an R out here is not a low saturation, it is a
         * measurement that has gone wrong: the channels reversed, a
         * non-pulsatile pedestal on IR, or common-mode interference such as
         * ambient light or movement, which inflates R because RED has the
         * smaller DC to divide by.
         *
         * This used to be handled by clamping sp to 700, which turned every
         * one of those failures into a confident, immovable "70 %" on the
         * display -- ppg.valid only asks that spo2_x10 be non-zero, so the
         * firmware actively asserted a reading it had no basis for.  Report
         * nothing instead, and raise a flag saying why. */
        if (r8 > R_TRUST_MAX) {
            ppg.spo2_rail = 1;
            ppg.spo2_x10  = 0;
            return;
        }
        ppg.spo2_rail = 0;
        if (r8 < 0)   r8 = 0;
        r2 = (r8 * r8) >> 8;
        /* SpO2 = -45.06*R^2 + 30.354*R + 94.845, evaluated in Q8 */
        sp = 24280L + ((7771L * r8) >> 8) - ((11535L * r2) >> 8);
        sp = (sp * 10) >> 8;                        /* -> tenths of a % */
        sp += cfg.spo2_cal;
        if (sp > 1000) sp = 1000;
        if (sp < 700)  sp = 700;
        ppg.spo2_x10 = (uint16_t)sp;
        if (!ppg.spo2_min_x10 || ppg.spo2_x10 < ppg.spo2_min_x10)
            ppg.spo2_min_x10 = ppg.spo2_x10;
    }
}

/* ---------------- one accepted beat ---------------- */
static void on_beat(uint16_t ibi_ms)
{
    ppg.ibi_ms  = ibi_ms;
    ppg.beats++;
    ppg.beat    = 1;
    ppg.beat_ms = millis();

    s.ibi[s.ibi_i] = ibi_ms;
    s.ibi_i = (uint8_t)((s.ibi_i + 1) % IBI_HIST);
    if (s.ibi_n < IBI_HIST) s.ibi_n++;

    /* BPM from the median interval: immune to one dropped or extra beat */
    {
        uint16_t med = median_u16(s.ibi, s.ibi_n);
        if (med) {
            uint16_t b = (uint16_t)(600000UL / med);
            if (b > 2500) b = 2500;
            ppg.bpm_x10 = b;
            if (s.ibi_n >= LOCK_BEATS) {
                if (!ppg.bpm_min_x10 || b < ppg.bpm_min_x10) ppg.bpm_min_x10 = b;
                if (b > ppg.bpm_max_x10) ppg.bpm_max_x10 = b;
                ppg.bpm_avg_x10 = ppg.bpm_avg_x10
                    ? (uint16_t)(ppg.bpm_avg_x10 + (((int32_t)b - ppg.bpm_avg_x10) >> 4))
                    : b;
            }
        }
    }

    spo2_update();
    hrv_update();
    sqi_update();

    ppg.progress = (uint8_t)((s.ibi_n >= LOCK_BEATS) ? 100
                                                     : (s.ibi_n * 100u) / LOCK_BEATS);
    /* Publication deliberately does NOT depend on SQI.  The quality figure
     * is penalised by the reject count, so a spell of rejected crossings
     * dragged it under the threshold and the reading then never appeared --
     * a gate that shuts harder the more it is needed.  SQI is shown to the
     * user as bars instead, which is what it is good for. */
    ppg.valid = (uint8_t)(s.ibi_n >= LOCK_BEATS && ppg.spo2_x10 != 0);
}

/* ---------------- respiration, updated every sample ---------------- */
static void resp_process(uint32_t ir)
{
    int32_t v1, v2, rs, a, hyst, floor_;

    if (!s.r_init) {
        s.r_lp1 = s.r_lp2 = (int32_t)(ir << RESP_LP_SHIFT);
        s.r_hp  = (int32_t)(ir << RESP_HP_SHIFT);
        s.r_init = 1;
    }
    s.r_lp1 += (int32_t)ir - (s.r_lp1 >> RESP_LP_SHIFT);
    v1 = s.r_lp1 >> RESP_LP_SHIFT;
    s.r_lp2 += v1 - (s.r_lp2 >> RESP_LP_SHIFT);
    v2 = s.r_lp2 >> RESP_LP_SHIFT;
    s.r_hp  += v2 - (s.r_hp >> RESP_HP_SHIFT);
    rs = v2 - (s.r_hp >> RESP_HP_SHIFT);

    a = (rs < 0) ? -rs : rs;
    if (a > s.r_env) s.r_env = a;
    else             s.r_env -= (s.r_env >> RESP_ENV_SHIFT) + 1;
    if (s.r_env < 0) s.r_env = 0;

    hyst   = s.r_env >> 3;
    floor_ = (int32_t)(ppg.dc_ir / RESP_FLOOR_DIV);
    if (floor_ < 4) floor_ = 4;
    if (s.r_env < floor_) s.r_low++;

    if (!s.r_above && rs >  hyst) { s.r_above = 1; s.r_cross++; }
    else if (s.r_above && rs < -hyst) { s.r_above = 0; }

    if (++s.r_win >= RESP_WIN) {
        /* breaths per minute = crossings / window_seconds * 60 */
        uint32_t rr = ((uint32_t)s.r_cross * ppg.fs_x100) / 5000UL;
        ppg.resp_bpm = (uint8_t)((s.r_low > (RESP_WIN / 2) || rr > 40 || rr < 4)
                                 ? 0 : rr);
        s.r_cross = s.r_win = s.r_low = 0;
    }
}

/* Samples the FIFO dropped never reach ppg_process(), yet real time still
 * passed.  Advance the counter over the gap and refuse to measure an interval
 * across it, otherwise the next beat is timed as if the gap never happened. */
void ppg_lost_samples(uint8_t n)
{
    if (!n) return;

    /* Advancing the counter is enough for a small gap: the time base stays
     * correct and a sample or two is inside the interpolation noise anyway.
     * Throwing away the beat reference for every loss was far too harsh --
     * the bus drops a byte often enough that a couple of samples go missing
     * roughly once a second, so nearly every interval was being discarded
     * and the median never had good data to work from. */
    s.nsamp += n;
    if (n < LOST_RESYNC) return;

    s.win_valid  = 0;
    s.t_cross_q8 = 0;
    s.refrac     = 0;
    ppg.rejects++;
}

/* ================= per-sample entry point ================= */
void ppg_process(uint32_t red, uint32_t ir)
{
    int32_t dc_ir, dc_red, ai, ar, f, fr, span;

    s.nsamp++;

    if (!s.dc_init) {
        s.dcacc_ir  = (int32_t)(ir  << DC_SHIFT);
        s.dcacc_red = (int32_t)(red << DC_SHIFT);
        s.dc_init   = 1;
        s.fprev     = 0;
    }
    s.dcacc_ir  += (int32_t)ir  - (s.dcacc_ir  >> DC_SHIFT);
    s.dcacc_red += (int32_t)red - (s.dcacc_red >> DC_SHIFT);
    dc_ir  = s.dcacc_ir  >> DC_SHIFT;
    dc_red = s.dcacc_red >> DC_SHIFT;
    if (dc_ir  < 1) dc_ir  = 1;
    if (dc_red < 1) dc_red = 1;
    ppg.dc_ir  = (uint32_t)dc_ir;
    ppg.dc_red = (uint32_t)dc_red;

    /* ---- finger presence: gain-normalised rise over the learned idle ---- */
    {
        uint32_t refl, on_th, off_th;
        uint8_t  want = ppg.finger;
        uint8_t  pa   = ppg.led_ir ? ppg.led_ir : 1;

        /* Normalise DC to the reference LED drive before comparing anything.
         * Reflection scales with drive, and once a finger is latched the AGC
         * walks the drive up by as much as 2x to optimise the AC signal -- so
         * a threshold derived from a baseline learned at the search drive
         * simply does not apply to the raw reading any more.  That is what
         * stopped a finger from ever being released: at the raised drive the
         * bare-sensor idle level sat ABOVE the release threshold computed for
         * the search drive, so lifting the finger still read as a finger and
         * the measurement and waveform ran on for ever.
         *
         * Comparing normalised values makes detection independent of the
         * gain, which is what it should have been all along -- it also
         * removes the need to scale the thresholds by the drive. */
        refl = ((uint32_t)dc_ir * LED_PA_REF) / pa;
        ppg.refl_ir = refl;

        if (!ppg.finger) {
            /* slow minimum tracker with a gentle leak upward, so it follows
             * ambient light without chasing a real finger */
            if (s.base_dc == 0 || refl < s.base_dc) {
                s.base_dc = refl;
                s.base_cnt = 0;
            } else if (++s.base_cnt >= 500) {
                s.base_cnt = 0;
                s.base_dc += (s.base_dc >> 5) + 4;
            }
        }

        /* The rise: additive plus a quarter of the idle level. */
        on_th = s.base_dc + (s.base_dc >> 2) + FINGER_MIN_RISE;

        /* Lower bound, then the reference upper bound (see the notes above:
         * a cap, so a poisoned baseline cannot push detection out of reach). */
        if (on_th < FINGER_FLOOR_IR) on_th = FINGER_FLOOR_IR;
        if (on_th > FINGER_CAP_IR)   on_th = FINGER_CAP_IR;
        /* Whatever the clamps did, the threshold has to stay above the idle
         * level or it would latch on nothing at all. */
        if (on_th < s.base_dc + (FINGER_MIN_RISE >> 2))
            on_th = s.base_dc + (FINGER_MIN_RISE >> 2);

        off_th = on_th - (on_th >> 2);              /* 25 % hysteresis */
        /* The release level has to clear the idle level too.  On a module
         * idling at 40000 the plain 25 % band lands at 37500 -- below what
         * the bare sensor reads -- so the first detection would never be
         * released. */
        if (off_th < s.base_dc + (FINGER_MIN_RISE >> 3))
            off_th = s.base_dc + (FINGER_MIN_RISE >> 3);
        if (off_th >= on_th) off_th = on_th - 1;

        ppg.finger_th = on_th;
        ppg.base_ir   = s.base_dc;

        /* After a no-pulse timeout, re-arming waits for the reading to come
         * back down past the release level, so a finger that is resting there
         * without a detectable pulse cannot latch again immediately and
         * oscillate on the timeout period. */
        if (s.no_arm && refl < off_th) s.no_arm = 0;

        if (!ppg.finger && !s.no_arm && refl > on_th) want = 1;
        if ( ppg.finger && refl < off_th)             want = 0;
        if (want != ppg.finger) {
            if (++s.finger_cnt > 30) {              /* ~0.3 s of agreement */
                ppg.finger   = want;
                s.finger_cnt = 0;
                ppg_reset_measure();
                if (want) ppg.sess_start_ms = millis();
                return;
            }
        } else {
            s.finger_cnt = 0;
        }
    }

    /* ---- band-pass 0.5 .. 5 Hz, identical on both channels ---- */
    ai = (int32_t)ir  - dc_ir;
    ar = (int32_t)red - dc_red;
    s.hp_ir  += ai - (s.hp_ir  >> HP2_SHIFT);
    ai       -= (s.hp_ir  >> HP2_SHIFT);
    s.hp_red += ar - (s.hp_red >> HP2_SHIFT);
    ar       -= (s.hp_red >> HP2_SHIFT);

    s.lp1  += (ai - s.lp1)  >> LP_SHIFT;
    s.lp2  += (s.lp1 - s.lp2) >> LP_SHIFT;
    s.lp1r += (ar - s.lp1r) >> LP_SHIFT;
    s.lp2r += (s.lp1r - s.lp2r) >> LP_SHIFT;
    f  = s.lp2;
    fr = s.lp2r;

    /* ---- dual envelope: threshold rides between trough and peak ---- */
    if (f > s.pk) s.pk = f;
    else          s.pk -= ((s.pk - s.tr) >> ENV_DECAY) + 1;
    if (f < s.tr) s.tr = f;
    else          s.tr += ((s.pk - s.tr) >> ENV_DECAY) + 1;
    if (s.pk < s.tr) s.pk = s.tr;
    span  = s.pk - s.tr;
    s.amp = span >> 1;

    /* ---- normalised trace for the display ---- */
    {
        int32_t den = (s.amp > AMP_FLOOR) ? s.amp : AMP_FLOOR;
        int32_t w   = ppg.finger ? ((f * 110) / den) : 0;
        if (w >  110) w =  110;
        if (w < -110) w = -110;
        ppg.wave = (int8_t)w;
        if (s.half ^= 1) {                          /* decimate by 2 */
            ppg_wave[ppg_wave_head] = ppg.wave;
            ppg_wave_head = (uint8_t)((ppg_wave_head + 1) % WAVE_LEN);
        }
    }

    if (s.settle)    { s.settle--;  s.fprev = f; return; }
    if (!ppg.finger) {              s.fprev = f; return; }

    resp_process(ir);

    /* ---- per-beat measurement windows ---- */
    if (!s.win_valid) {
        s.fir_min  = s.fir_max  = f;
        s.fred_min = s.fred_max = fr;
        s.rir_min  = s.rir_max  = ir;
        s.rred_min = s.rred_max = red;
        s.ssq_ir = s.ssq_red = 0;
        s.sxy    = 0;
        s.ssq_n  = 0;
        s.win_valid = 1;
    } else {
        if (f  > s.fir_max)  s.fir_max  = f;
        if (f  < s.fir_min)  s.fir_min  = f;
        if (fr > s.fred_max) s.fred_max = fr;
        if (fr < s.fred_min) s.fred_min = fr;
        if (ir  > s.rir_max)  s.rir_max  = ir;
        if (ir  < s.rir_min)  s.rir_min  = ir;
        if (red > s.rred_max) s.rred_max = red;
        if (red < s.rred_min) s.rred_min = red;
    }

    /* Sums of squares and the cross product, over the same window.  Both
     * channels are shifted and clamped identically, so the scaling cancels
     * in the ratio and in the correlation; the cap on the window keeps the
     * worst case inside a uint32 (2047^2 * 512 = 2.1e9). */
    if (s.ssq_n < RMS_N_MAX) {
        int32_t qi = f  >> RMS_SHIFT;
        int32_t qr = fr >> RMS_SHIFT;
        if (qi >  RMS_Q_MAX) qi =  RMS_Q_MAX;
        if (qi < -RMS_Q_MAX) qi = -RMS_Q_MAX;
        if (qr >  RMS_Q_MAX) qr =  RMS_Q_MAX;
        if (qr < -RMS_Q_MAX) qr = -RMS_Q_MAX;
        s.ssq_ir  += (uint32_t)(qi * qi);
        s.ssq_red += (uint32_t)(qr * qr);
        s.sxy     += qi * qr;
        s.ssq_n++;
    }

    /* ---- per-cycle peak and trough, reset at each beat ---- */
    if (f > s.cyc_max) s.cyc_max = f;
    if (f < s.cyc_min) s.cyc_min = f;

    /* ---- refractory period: nothing counts for a while after a beat ---- */
    if (s.refrac) {
        s.refrac--;
        s.fprev = f;
        return;
    }

    /* ---- beat detection: upward ZERO crossing ----
     * This replaces a threshold set at 5/8 of a decaying peak-to-trough
     * envelope.  That fires at a level which moves with the pulse height, so
     * it is defeated by the two things every PPG has: beat-to-beat amplitude
     * variation from breathing, and the dicrotic notch on the falling edge,
     * which re-armed the detector and fired a second time in the same beat.
     *
     * A band-passed PPG crosses zero upward exactly ONCE per cardiac cycle
     * whatever its amplitude, and the notch sits on the negative side without
     * crossing back.  That is why the reference implementation keys on the
     * zero crossing (SparkFun heartRate.cpp checkForBeat(), which is what the
     * SunFounder lesson calls), and it is immune to both problems by
     * construction.  Amplitude is used only to reject noise, exactly as the
     * reference does with its IR_AC_Max - IR_AC_Min test. */
    if (s.fprev < 0 && f >= 0) {
        uint32_t t_q8;
        int32_t  den = f - s.fprev;
        int32_t  pp  = s.cyc_max - s.cyc_min;   /* amplitude of the cycle */
        uint16_t frac = 0;

        if (den > 0) {
            int32_t num = -s.fprev;             /* distance up to zero */
            if (num < 0)   num = 0;
            if (num > den) num = den;
            frac = (uint16_t)(((uint32_t)num << 8) / den);
        }
        t_q8 = (((uint32_t)(s.nsamp - 1)) << 8) + frac;
        s.cyc_max = s.cyc_min = f;              /* begin the next cycle */

        /* Noise gate, and nothing more.  The reference accepts any cycle
         * between 20 and 1000 on its own scale -- a 50:1 window -- and
         * applies no other amplitude test whatsoever.  Being stricter than
         * that is what stopped beats being accepted at all: a narrow band
         * around a running average locks out permanently the moment one
         * unrepresentative beat sets that average, and a hard perfusion-index
         * ceiling throws away the strong pulse a well-placed finger actually
         * gives.  So reject only a flat trace, keep the window wide, track
         * the average on EVERY crossing so it can never get stuck, and let
         * the median downstream deal with the outliers.
         *
         * The average is updated BEFORE the window test, which is what
         * "on EVERY crossing" has to mean for the claim above to hold.  With
         * the update below the early return it only ever saw crossings that
         * had already passed, so one unrepresentative first crossing latched
         * it and locked the detector out permanently: settling right after a
         * finger latches produces a crossing of a few tens of counts, that
         * set amp_avg to ~36, and every real beat at ~1000 counts then failed
         * pp > (amp_avg << 3) = 288 for ever.  No beats means the 15 s
         * no-pulse timeout drops the finger and arms s.no_arm, which will not
         * clear while the finger is still on the sensor -- so the device sat
         * there with a perfect trace, reporting nothing, until it was lifted.
         * Whether the first crossing landed in the fatal band (above
         * AMP_MIN_BEAT but under an eighth of the real pulse) came down to
         * noise, which is what made it intermittent. */
        s.amp_avg = s.amp_avg ? (s.amp_avg + ((pp - s.amp_avg) >> 3)) : pp;

        if (pp < AMP_MIN_BEAT ||
            (pp < (s.amp_avg >> 3) || pp > (s.amp_avg << 3))) {
            dbg_beat(0, (uint16_t)((pp > 65535) ? 65535 : pp), BEAT_AMP);
            s.fprev = f;
            return;                             /* keeps the reference time */
        }

        if (s.t_cross_q8) {
            uint32_t dt_q8  = t_q8 - s.t_cross_q8;
            uint16_t ibi_ms;
            uint8_t  code = BEAT_OK;

            /* dt_q8 * 25000 overflows a uint32 past dt_q8 = 171798, which is
             * 671 samples -- only 5.4 s at the measured 123 Hz.  A longer gap
             * between crossings wrapped the product and produced a RANDOM
             * interval, and because the continuity check below only runs once
             * four intervals are banked, an early wrapped value landing
             * anywhere in 300..2000 ms was accepted unchecked and poisoned
             * the median the rate is taken from.
             *
             * Gaps that long are routine at the start of a measurement: the
             * AGC steps the drive several times while DC settles and each
             * step blanks the detector.  Clamping well inside the overflow
             * keeps the arithmetic honest -- the result then exceeds
             * IBI_MAX_MS and is rejected, which is the correct outcome. */
            if (dt_q8 > 0x20000UL) dt_q8 = 0x20000UL;      /* ~4.2 s */
            ibi_ms = (uint16_t)(((dt_q8 * 25000UL) >> 6) / ppg.fs_x100);

            /* The interval bounds ARE the acceptance test, as in the
             * reference: it takes any rate between 20 and 255 bpm and applies
             * no further check.  A very loose continuity test is kept on top,
             * but only once a real history exists, so it can catch a gross
             * jump without ever being able to block acquisition.
             *
             * What used to be here -- an acquisition gate demanding the first
             * intervals agree within 25 %, then a +-25 % continuity band, then
             * a wipe after eight rejects -- could reject every candidate and
             * leave the rate at zero for ever.  Layers of plausible-looking
             * validation, each able to veto, is how a working measurement
             * turns into no measurement at all. */
            if      (ibi_ms < IBI_MIN_MS) code = BEAT_SHORT;
            else if (ibi_ms > IBI_MAX_MS) code = BEAT_LONG;
            else if (s.ibi_n >= 8) {
                uint16_t med = median_u16(s.ibi, s.ibi_n);
                uint16_t tol = (uint16_t)(med >> 1);        /* +-50 % */
                if (ibi_ms > (uint16_t)(med + tol) || ibi_ms + tol < med)
                    code = BEAT_CONT;
            }
            dbg_beat(ibi_ms, (uint16_t)((pp > 65535) ? 65535 : pp), code);

            if (code == BEAT_OK) {
                on_beat(ibi_ms);
            } else {
                ppg.rejects++;
                /* Too SHORT means this crossing was noise rather than the
                 * beat being missed, so keep the last good reference. */
                if (code == BEAT_SHORT) { s.fprev = f; return; }
            }
        }
        s.t_cross_q8 = t_q8;
        s.win_valid  = 0;                           /* start a fresh AC window */
        /* Safety net only.  With a zero crossing there is exactly one per
         * cardiac cycle, so the dicrotic notch cannot fire a second time and
         * the hold-off no longer has to be long enough to straddle it -- it
         * just has to be shorter than any real interval. */
        {
            /* Hold-off proportional to the rate once one is known.  A flat
             * 200 ms floor is all the reference needs because its 23-tap FIR
             * smooths the dicrotic notch away entirely; two 1-pole sections
             * leave more of it, and the notch lands 250-400 ms after the
             * systolic peak -- past a flat floor, so it got timed as a beat
             * and the rate came out roughly doubled.  60 % of the median
             * covers it while still allowing the rate to rise by two thirds
             * between beats. */
            uint32_t hold = REFRAC_MS;
            if (s.ibi_n >= 4) {
                uint32_t adaptive = ((uint32_t)median_u16(s.ibi, s.ibi_n) * 3) / 5;
                if (adaptive > hold) hold = adaptive;
            }
            if (hold > REFRAC_MAX) hold = REFRAC_MAX;
            s.refrac = (uint16_t)((hold * ppg.fs_x100) / 100000UL);
        }
    }
    s.fprev = f;
}

/* ================= housekeeping, called from the main loop ================= */
static void agc_step(void)
{
    uint8_t nr = ppg.led_red, ni = ppg.led_ir;

    if (cfg.led_mode != LED_MODE_AUTO) {
        nr = ni = led_fixed_pa();
    } else if (!ppg.finger) {
        /* Searching: hold the reference drive.  This used to ramp towards 0x60
         * whenever idle DC read low, which is self-defeating -- the finger
         * threshold is an absolute count calibrated at LED_PA_REF, and a drive
         * that drifts underneath it makes the comparison meaningless.  Pinning
         * it also gives the baseline tracker a stationary level to learn.
         * A finger at 0x1F reads well past 50000; the AGC gets to optimise the
         * gain for signal quality once there is actually something to measure. */
        nr = ni = LED_PA_SEARCH;
    } else {
        /* Finger present: park DC comfortably above the release threshold
         * (FINGER_OFF_IR) so a gain step can never shake the finger off, and
         * below saturation.  Step harder when far outside the window: at two
         * counts per 250 ms a full-scale correction took twelve seconds, which
         * is longer than the whole six-beat acquisition. */
        uint8_t step = 2;
        if (ppg.dc_ir < 40000UL || ppg.dc_ir > 230000UL) step = 6;

        if      (ppg.dc_ir  <  60000UL && ni < LED_PA_MAX) ni = (uint8_t)(ni + step);
        else if (ppg.dc_ir  > 200000UL && ni > LED_PA_MIN) ni = (uint8_t)(ni - step);
        if      (ppg.dc_red <  60000UL && nr < LED_PA_MAX) nr = (uint8_t)(nr + step);
        else if (ppg.dc_red > 200000UL && nr > LED_PA_MIN) nr = (uint8_t)(nr - step);
    }
    if (nr > LED_PA_MAX) nr = LED_PA_MAX;
    if (ni > LED_PA_MAX) ni = LED_PA_MAX;
    if (nr < LED_PA_MIN) nr = LED_PA_MIN;
    if (ni < LED_PA_MIN) ni = LED_PA_MIN;

    if (nr != ppg.led_red || ni != ppg.led_ir) {
        ppg.led_red = nr;
        ppg.led_ir  = ni;
        max30102_set_leds(nr, ni);
        s.settle = SETTLE_SAMPLES;    /* readings during the step are discarded */
        /* The blanking window swallows any crossing inside it, so an
         * interval measured across a gain step spans an unknown number of
         * missed beats.  Drop the reference rather than time through it. */
        s.t_cross_q8 = 0;
        s.refrac     = 0;
    }
}

void ppg_service(void)
{
    uint32_t now = millis();

    if ((uint32_t)(now - s.agc_ms) >= 250) { s.agc_ms = now; agc_step(); }

    if ((uint32_t)(now - s.temp_ms) >= 5000) {
        s.temp_ms = now;
        max30102_temp_start();
    }
    {
        int16_t t;
        if (max30102_temp_ready(&t)) ppg.temp_x10 = t;
    }

    /* Raw arrival rate, reported honestly: if the FIFO is giving us nothing
     * this reads 0 rather than the nominal rate, which is the difference
     * between "the sensor is streaming" and "the sensor is silent". */
    if ((uint32_t)(now - s.sps_ms) >= 1000) {
        uint32_t d = s.nsamp - s.sps_mark;
        ppg.sps    = (uint16_t)(d > 65535UL ? 65535UL : d);
        s.sps_mark = s.nsamp;
        s.sps_ms   = now;
    }

    /* Calibrate the true ADC output rate against the 16 MHz crystal.  The
     * MAX30102's internal oscillator is only good to a few percent -- this
     * board measures about 118 Hz where 100 is nominal -- and because beats
     * are timed by counting samples, that error lands directly on the BPM.
     *
     * Estimated over one long baseline rather than a 4 s sliding window fed
     * through an IIR.  That arrangement was measured still climbing, 104 ->
     * 118, twenty seconds after start-up: readings taken while it settled
     * were wrong by up to 15 % and drifted the whole time.  A single long
     * interval converges as 1/N and is stable within a few seconds. */
    if ((uint32_t)(now - s.fs_ms) >= 1000) {
        uint32_t dn = s.nsamp - s.fs_nmark;
        uint32_t dt = now - s.fs_t0;
        s.fs_ms = now;

        if (ppg.sps < 50) {
            /* Stream stalled or not started: measuring across the gap would
             * read as a collapse in sample rate.  Restart the baseline. */
            s.fs_t0    = now;
            s.fs_nmark = s.nsamp;
        } else if (dt >= 3000 && dn > 200) {
            uint32_t fs = (dn * 100000UL) / dt;     /* Hz x100 */
            if (fs > 3000UL && fs < 60000UL) ppg.fs_x100 = (uint16_t)fs;

            /* Once the baseline is long enough for the estimate to be solid,
             * store it so the next power-up starts correct instead of
             * spending the first measurement converging.  Written at most
             * once per session, and only if it actually moved, because the
             * EEPROM has a finite write endurance. */
            if (!s.fs_saved && dt > 20000UL) {
                uint16_t d = (uint16_t)((cfg.fs_cal > ppg.fs_x100)
                                        ? (cfg.fs_cal - ppg.fs_x100)
                                        : (ppg.fs_x100 - cfg.fs_cal));
                s.fs_saved = 1;
                if (!cfg.fs_cal || d > (ppg.fs_x100 / 50)) {      /* > 2 %  */
                    cfg.fs_cal = ppg.fs_x100;
                    settings_save();
                }
            }
            /* Restart occasionally so a real change -- a new averaging
             * setting, a warming oscillator -- is picked up rather than
             * averaged away for ever. */
            if (dt > 120000UL) { s.fs_t0 = now; s.fs_nmark = s.nsamp; }
        }
    }

    /* drop a stale reading if beats stop arriving */
    if (ppg.valid && (uint32_t)(now - ppg.beat_ms) > 4000) {
        ppg.valid    = 0;
        ppg.progress = 0;
    }

    /* Something that produces no pulse at all is not a finger.  The DC test
     * alone cannot know that, so anything that clears the threshold and stays
     * there -- a fingertip resting beside the window, a bright ambient
     * source, a smudge on the glass -- would keep the session alive and the
     * waveform scrolling indefinitely.  Give acquisition a fair window, then
     * let go and re-learn the idle level. */
    if (ppg.finger &&
        (uint32_t)(now - ppg.sess_start_ms) > FINGER_NOBEAT_MS &&
        (uint32_t)(now - ppg.beat_ms)       > FINGER_NOBEAT_MS) {
        ppg.finger   = 0;
        s.finger_cnt = 0;
        s.no_arm     = 1;          /* wait for the reading to fall back first */
        ppg_reset_measure();
    }
}
