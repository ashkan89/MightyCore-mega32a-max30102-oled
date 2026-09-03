/* ------------------------------------------------------------------
 *  tests/host/test_ppg.c -- host-side tests for the PPG signal chain
 *
 *  These exercise the REAL ppg.c and sys.c, not a reimplementation:
 *  both are #included below so the test shares their translation unit
 *  and can drive the millisecond counter directly.  Everything AVR is
 *  supplied by tests/host/stub/, and STACK_GUARD is 0 so the one piece
 *  of inline assembly in the firmware is compiled out.
 *
 *  What this can and cannot prove:
 *
 *    it CAN   verify the integer maths, the beat detector against a
 *             synthetic PPG of known rate, the acceptance and rejection
 *             gates, the finger state machine, staleness handling and
 *             millis() wraparound -- all of it deterministic and
 *             repeatable
 *
 *    it CANNOT say anything about optical accuracy, LED currents, I2C
 *             behaviour, timing on the real part, or whether the SpO2
 *             calibration curve matches a reference oximeter.  Those
 *             need the hardware; see README, "On-device validation".
 *
 *  Build and run:  tests/host/run.cmd
 * ------------------------------------------------------------------ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* ---- host build configuration, before anything includes config.h ---- */
#define STACK_GUARD 0        /* compiles out the inline asm */
#define DBG_MODE    0        /* dbg.h then supplies inline no-ops */

/* ---- the fake I/O space declared by the stub avr/io.h ---- */
uint8_t H_SREG, H_MCUCSR;
uint8_t H_TCCR0, H_OCR0, H_TIMSK, H_TCCR2, H_OCR2, H_TCNT2;
uint8_t H_PORTB, H_DDRB, H_PINB;
uint8_t H_PORTC, H_DDRC, H_PINC;
uint8_t H_PORTD, H_DDRD, H_PIND;

#include "settings.h"
#include "max30102.h"

/* ---- collaborators the DSP calls but that are not under test ---- */
settings_t cfg;

static uint16_t stub_led_red, stub_led_ir;
static int      stub_saves;

void max30102_set_leds(uint8_t red_pa, uint8_t ir_pa)
{
    stub_led_red = red_pa;
    stub_led_ir  = ir_pa;
}
void max30102_temp_start(void) { }
uint8_t max30102_temp_ready(int16_t *t) { (void)t; return 0; }
void settings_save(void) { stub_saves++; }

/* ---- the code under test ---- */
#include "sys.c"
#include "ppg.c"

/* ---- forward declarations ---- */
static void settings_defaults_host(void);

/* ==================================================================
 *  Test scaffolding
 * ================================================================== */
static int g_fail, g_checks;

#define CHECK(cond, ...)                                                  \
    do {                                                                  \
        g_checks++;                                                       \
        if (!(cond)) {                                                    \
            g_fail++;                                                     \
            printf("  FAIL %s:%d  ", __FILE__, __LINE__);                 \
            printf(__VA_ARGS__);                                          \
            printf("\n");                                                 \
        }                                                                 \
    } while (0)

static void banner(const char *name) { printf("\n== %s ==\n", name); }

/* ------------------------------------------------------------------
 *  Synthetic PPG generator
 *
 *  A cardiac cycle as two Gaussians: the systolic peak, and a dicrotic
 *  notch at 35 % of its height a third of a cycle later.  The notch is
 *  the whole point -- it is what a threshold detector fires twice on,
 *  and the zero-crossing detector must not.  Normalised to unit
 *  peak-to-peak, mean removed, so the caller sets the AC amplitude.
 * ------------------------------------------------------------------ */
static double ppg_shape(double u)          /* u in [0,1) */
{
    double a = (u - 0.15) / 0.09;
    double b = (u - 0.42) / 0.13;
    return exp(-a * a) + 0.35 * exp(-b * b);
}

static double shape_mean, shape_span;

static void shape_calibrate(void)
{
    double lo = 1e9, hi = -1e9, sum = 0.0;
    int i;
    for (i = 0; i < 1000; i++) {
        double v = ppg_shape(i / 1000.0);
        if (v < lo) lo = v;
        if (v > hi) hi = v;
        sum += v;
    }
    shape_span = hi - lo;
    shape_mean = sum / 1000.0;
}

static double shape_ac(double u)           /* zero-mean, unit span */
{
    return (ppg_shape(u) - shape_mean) / shape_span;
}

/* ------------------------------------------------------------------
 *  Simulator
 *
 *  Feeds ppg_process() at a chosen sample rate, advancing the same
 *  millisecond counter the firmware reads, and calling ppg_service() on
 *  the same 250 ms cadence the main loop does.
 * ------------------------------------------------------------------ */
typedef struct {
    double bpm;            /* heart rate to synthesise             */
    double dc_ir, dc_red;  /* DC levels in ADC counts              */
    double ac_ir, ac_red;  /* AC peak-to-peak in ADC counts        */
    double noise;          /* uniform noise amplitude, counts      */
    int    fs;             /* samples per second                   */
} sim_t;

static double sim_phase;
static unsigned long sim_seed = 12345u;

static double sim_rand(void)              /* -1 .. +1, repeatable */
{
    sim_seed = sim_seed * 1103515245u + 12345u;
    return ((double)((sim_seed >> 16) & 0x7FFF) / 16383.5) - 1.0;
}

static void sim_reset(void) { sim_phase = 0.0; sim_seed = 12345u; }

/* Runs the simulator for ms_total milliseconds. */
static void sim_run(const sim_t *p, uint32_t ms_total)
{
    uint32_t step_us = (uint32_t)(1000000UL / (uint32_t)p->fs);
    uint32_t us_acc  = 0;
    uint32_t n       = (uint32_t)(((double)ms_total * p->fs) / 1000.0);
    uint32_t i;

    for (i = 0; i < n; i++) {
        double a = shape_ac(sim_phase);
        double ir  = p->dc_ir  + a * p->ac_ir  + sim_rand() * p->noise;
        double red = p->dc_red + a * p->ac_red + sim_rand() * p->noise;
        if (ir  < 0) ir  = 0;
        if (red < 0) red = 0;
        if (ir  > 262143.0) ir  = 262143.0;   /* 18-bit ADC */
        if (red > 262143.0) red = 262143.0;

        ppg_process((uint32_t)red, (uint32_t)ir);

        sim_phase += p->bpm / (60.0 * p->fs);
        if (sim_phase >= 1.0) sim_phase -= 1.0;

        /* advance the firmware's clock */
        us_acc += step_us;
        while (us_acc >= 1000) { us_acc -= 1000; g_ms++; }

        /* The main loop calls this every pass; 250 ms is the interval
         * that actually matters, because that is the AGC cadence. */
        if ((i % (uint32_t)(p->fs / 4)) == 0) ppg_service();
    }
}

/* Brings the DSP up with a known configuration. */
static void sim_init(void)
{
    settings_defaults_host();
    ppg_init();
    sim_reset();
}

/* cfg is normally owned by settings.c, which is not compiled here. */
static void settings_defaults_host(void)
{
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic     = 0xA5;
    cfg.version   = SETTINGS_VERSION;
    cfg.contrast  = 0xCF;
    cfg.led_mode  = LED_MODE_AUTO;
    cfg.avg_code  = 2;              /* 4x -> 100 Hz FIFO output */
    cfg.spo2_cal  = 0;
    cfg.sleep_s   = 120;
    cfg.fs_cal    = 0;
    cfg.probe_v   = PROBE_UNKNOWN;
}

/* ==================================================================
 *  1. Integer helpers
 * ================================================================== */
static void test_isqrt(void)
{
    uint32_t v;
    uint32_t i;                 /* not int: 16 bits wide on the target */
    banner("isqrt32");

    CHECK(isqrt32(0) == 0, "isqrt32(0)=%u", isqrt32(0));
    CHECK(isqrt32(1) == 1, "isqrt32(1)=%u", isqrt32(1));

    /* Exact squares must come back exactly, and every result must
     * satisfy r*r <= n < (r+1)*(r+1) -- the definition of the integer
     * square root, which is what the callers rely on. */
    for (i = 0; i <= 65535UL; i += 7) {
        v = i * i;
        CHECK(isqrt32(v) == (uint16_t)i, "isqrt32(%lu)=%u want %lu",
              (unsigned long)v, isqrt32(v), (unsigned long)i);
    }
    for (v = 0; v < 4000000UL; v += 3571) {
        uint32_t r = isqrt32(v);
        CHECK(r * r <= v && (uint64_t)(r + 1) * (r + 1) > v,
              "isqrt32(%lu)=%lu out of bounds",
              (unsigned long)v, (unsigned long)r);
    }
    /* The largest value the correlation gate can present. */
    CHECK(isqrt32(0xFFFFFFFFUL) == 65535, "isqrt32(UINT32_MAX)=%u",
          isqrt32(0xFFFFFFFFUL));
}

static int cmp_u16(const void *a, const void *b)
{
    uint16_t x = *(const uint16_t *)a, y = *(const uint16_t *)b;
    return (x > y) - (x < y);
}

static void test_median(void)
{
    uint16_t buf[20], ref[20];
    uint8_t n;
    int t;
    banner("median_u16");

    CHECK(median_u16(buf, 0) == 0, "median of nothing must be 0");

    for (t = 0; t < 400; t++) {
        for (n = 1; n <= 12; n++) {
            uint8_t i;
            for (i = 0; i < n; i++)
                buf[i] = (uint16_t)(rand() % 3000);
            memcpy(ref, buf, n * sizeof(uint16_t));
            qsort(ref, n, sizeof(uint16_t), cmp_u16);
            CHECK(median_u16(buf, n) == ref[n >> 1],
                  "median n=%u got %u want %u", n, median_u16(buf, n),
                  ref[n >> 1]);
        }
    }

    /* The input must not be disturbed: the beat detector takes the
     * median of the live interval ring three times per beat and would
     * be reordering its own history if this sorted in place. */
    {
        uint16_t a[6] = { 900, 100, 500, 300, 700, 200 };
        uint16_t b[6];
        memcpy(b, a, sizeof(a));
        (void)median_u16(a, 6);
        CHECK(memcmp(a, b, sizeof(a)) == 0, "median_u16 modified its input");
    }

    /* n above the internal buffer is clamped, not overrun.  IBI_HIST is
     * 12 today; if it ever grows past the 12-element scratch array this
     * test is what notices. */
    CHECK(IBI_HIST <= 12, "IBI_HIST=%d exceeds median_u16's scratch array",
          IBI_HIST);
    CHECK(R_HIST <= 12, "R_HIST=%d exceeds median_u16's scratch array",
          R_HIST);
    {
        uint8_t i;
        for (i = 0; i < 20; i++) buf[i] = (uint16_t)(i + 1);
        CHECK(median_u16(buf, 20) == 7, "clamped median got %u want 7",
              median_u16(buf, 20));
    }
}

/* ==================================================================
 *  2. The SpO2 calibration curve
 * ================================================================== */
static void test_spo2_curve(void)
{
    int r8;
    banner("SpO2 curve (fixed point vs float)");

    /* The firmware evaluates -45.06*R^2 + 30.354*R + 94.845 in Q8.
     * Check the fixed-point evaluation against the same polynomial in
     * double over the whole domain the curve is trusted on. */
    for (r8 = 0; r8 <= R_TRUST_MAX; r8++) {
        int32_t sp = 24280L + ((7771L * r8) >> 8)
                            - ((11535L * r8 * r8) >> 16);
        int32_t got = (sp * 10) >> 8;
        double  R   = r8 / 256.0;
        double  want = (-45.06 * R * R + 30.354 * R + 94.845) * 10.0;
        double  err = fabs((double)got - want);
        CHECK(err < 1.5, "R=%.4f fixed=%ld float=%.1f err=%.2f tenths",
              R, (long)got, want, err);
    }

    /* R_TRUST_MAX is meant to be the point where the curve passes 70 %,
     * the bottom of Maxim's table.  If someone moves it, this says so. */
    {
        double R = R_TRUST_MAX / 256.0;
        double sp = -45.06 * R * R + 30.354 * R + 94.845;
        CHECK(sp > 69.0 && sp < 71.5,
              "R_TRUST_MAX=%d gives %.2f %%, expected ~70", R_TRUST_MAX, sp);
    }

    /* The curve must be monotonically falling across the trusted range;
     * a maximum inside it would make two saturations share one R. */
    {
        int32_t prev = 0x7FFFFFFF;
        int ok = 1;
        for (r8 = 0; r8 <= R_TRUST_MAX; r8++) {
            int32_t sp = 24280L + ((7771L * r8) >> 8)
                                - ((11535L * r8 * r8) >> 16);
            if (r8 > 87 && sp > prev) ok = 0;   /* past the turning point */
            prev = sp;
        }
        CHECK(ok, "SpO2 curve is not monotonic over the trusted R range");
    }
}

/* ==================================================================
 *  3. Beat detection on a synthetic PPG
 * ================================================================== */
static void test_beat_rate(void)
{
    static const double rates[] = { 45.0, 60.0, 75.0, 100.0, 140.0, 180.0 };
    unsigned k;
    banner("beat detection: rate accuracy");

    for (k = 0; k < sizeof(rates) / sizeof(rates[0]); k++) {
        sim_t p;
        double got, err;

        p.bpm = rates[k];
        p.dc_ir = 90000; p.dc_red = 65000;
        p.ac_ir = 900;   p.ac_red = 360;     /* PI ~1 %, R ~0.6 */
        p.noise = 0;     p.fs = 100;

        sim_init();
        /* settle the idle baseline with no finger, then present one */
        {
            sim_t idle = p;
            idle.dc_ir = 2000; idle.dc_red = 1500;
            idle.ac_ir = 0;    idle.ac_red = 0;
            sim_run(&idle, 1500);
        }
        CHECK(ppg.finger == 0, "%.0f bpm: finger latched with no finger",
              rates[k]);

        sim_run(&p, 25000);

        CHECK(ppg.finger == 1, "%.0f bpm: finger not detected", rates[k]);
        CHECK(ppg.valid == 1, "%.0f bpm: never converged (beats=%lu rej=%lu)",
              rates[k], (unsigned long)ppg.beats, (unsigned long)ppg.rejects);

        got = ppg.bpm_x10 / 10.0;
        err = fabs(got - rates[k]) / rates[k] * 100.0;
        CHECK(err < 4.0, "%.0f bpm: measured %.1f (%.1f %% error)",
              rates[k], got, err);

        /* The dicrotic notch must not be counted.  Firing twice per beat
         * would put the rate near double and the reject count high. */
        CHECK(got < rates[k] * 1.5,
              "%.0f bpm: measured %.1f -- detector is double-firing",
              rates[k], got);
        CHECK(got > rates[k] * 0.6,
              "%.0f bpm: measured %.1f -- detector is missing beats",
              rates[k], got);
    }
}

static void test_spo2_end_to_end(void)
{
    /* Three red AC amplitudes spanning the useful part of the curve.
     * The expected saturation is NOT written down: it is computed from
     * the ratio the simulator constructs, so the test cannot drift away
     * from the polynomial it is checking, and editing the polynomial
     * without meaning to shows up here rather than in a stale constant.
     *
     *     R = (AC_red / DC_red) / (AC_ir / DC_ir)
     *
     * and the band-pass applies the identical transfer function to both
     * channels, so R survives filtering even though the amplitudes do
     * not. */
    static const double acr_cases[] = { 240.0, 360.0, 480.0, 600.0 };
    unsigned k;
    banner("SpO2 end to end");

    for (k = 0; k < sizeof(acr_cases) / sizeof(acr_cases[0]); k++) {
        sim_t p;
        double R, want, got;

        p.bpm = 72.0;
        p.dc_ir = 90000; p.dc_red = 65000;
        p.ac_ir = 900;   p.ac_red = acr_cases[k];
        p.noise = 0;     p.fs = 100;

        R    = (p.ac_red / p.dc_red) / (p.ac_ir / p.dc_ir);
        want = -45.06 * R * R + 30.354 * R + 94.845;

        sim_init();
        {
            sim_t idle = p;
            idle.dc_ir = 2000; idle.dc_red = 1500;
            idle.ac_ir = 0;    idle.ac_red = 0;
            sim_run(&idle, 1500);
        }
        sim_run(&p, 25000);

        CHECK(ppg.spo2_x10 != 0,
              "R=%.3f: no SpO2 published (rail=%u corr=%u)",
              R, ppg.spo2_rail, ppg.corr_x100);
        if (!ppg.spo2_x10) continue;

        got = ppg.spo2_x10 / 10.0;

        /* The firmware clamps its output to 70..100 %, so a case whose
         * true value sits outside that can only be checked against the
         * clamp. */
        if (want >= 100.0) {
            CHECK(got >= 99.0, "R=%.3f: want the 100 %% clamp, got %.1f",
                  R, got);
        } else {
            /* 2.5 points.  R is preserved by the filtering in principle,
             * but the peak-to-peak span of a filtered beat is still a
             * quantised estimate of it, and the weaker red channel is
             * the one that quantises worst. */
            CHECK(fabs(got - want) < 2.5,
                  "R=%.3f: SpO2 %.1f want %.1f (measured R=%.3f corr=%u)",
                  R, got, want, ppg.r_q12 / 4096.0, ppg.corr_x100);
        }

        /* R itself should come back close to what was constructed --
         * this is the check that would catch a reversed channel pair,
         * which returns 1/R. */
        CHECK(fabs(ppg.r_q12 / 4096.0 - R) < 0.12,
              "R=%.3f: firmware measured %.3f", R, ppg.r_q12 / 4096.0);

        CHECK(ppg.corr_x100 >= 80,
              "R=%.3f: correlation %u on a clean synthetic signal",
              R, ppg.corr_x100);
        CHECK(ppg.spo2_rail == 0, "R=%.3f: rail=%u", R, ppg.spo2_rail);

        /* Perfusion index: AC/DC on the IR channel, as a percentage. */
        {
            double pi_want = p.ac_ir / p.dc_ir * 100.0;
            double pi_got  = ppg.pi_x100 / 100.0;
            CHECK(fabs(pi_got - pi_want) < 0.4,
                  "R=%.3f: PI %.2f %% want %.2f %%", R, pi_got, pi_want);
        }
    }
}

/* ==================================================================
 *  4. Presence, rejection and staleness
 * ================================================================== */
static void test_no_finger(void)
{
    sim_t p;
    banner("no finger");

    p.bpm = 72.0;
    p.dc_ir = 1800; p.dc_red = 1400;
    p.ac_ir = 4;    p.ac_red = 2;
    p.noise = 3;    p.fs = 100;

    sim_init();
    sim_run(&p, 20000);

    CHECK(ppg.finger == 0, "finger latched on a bare sensor");
    CHECK(ppg.bpm_x10 == 0, "bpm %u published with no finger", ppg.bpm_x10);
    CHECK(ppg.spo2_x10 == 0, "SpO2 %u published with no finger", ppg.spo2_x10);
    CHECK(ppg.valid == 0, "valid asserted with no finger");
    CHECK(ppg.beats == 0, "%lu beats counted with no finger",
          (unsigned long)ppg.beats);
}

static void test_finger_release(void)
{
    sim_t p, idle;
    banner("finger removal");

    p.bpm = 72.0;
    p.dc_ir = 90000; p.dc_red = 65000;
    p.ac_ir = 900;   p.ac_red = 360;
    p.noise = 0;     p.fs = 100;

    idle = p;
    idle.dc_ir = 2000; idle.dc_red = 1500;
    idle.ac_ir = 0;    idle.ac_red = 0;

    sim_init();
    sim_run(&idle, 1500);
    sim_run(&p, 20000);
    CHECK(ppg.finger == 1, "finger not detected");
    CHECK(ppg.valid == 1, "did not converge before removal");

    /* Lift it.  Detection must clear, and the reading must not be left
     * standing on the display afterwards. */
    sim_run(&idle, 4000);
    CHECK(ppg.finger == 0, "finger not released (refl=%lu off_th<%lu)",
          (unsigned long)ppg.refl_ir, (unsigned long)ppg.finger_th);
    CHECK(ppg.valid == 0, "reading still valid after removal");
    CHECK(ppg.sqi == 0, "quality %u still shown after removal", ppg.sqi);
}

static void test_hr_without_spo2(void)
{
    sim_t p;
    uint32_t i;
    banner("heart rate is published without SpO2");

    /* A pulse on IR, and RED carrying noise that does not track it --
     * a weak or unseen red return, which is a normal outcome.  The
     * correlation gate must refuse SpO2, and the heart rate must still
     * be published: it used to be withheld with it, which left a device
     * tracking the pulse perfectly while reporting nothing. */
    p.bpm = 72.0;
    p.dc_ir = 90000; p.dc_red = 65000;
    p.ac_ir = 900;   p.ac_red = 0;
    p.noise = 0;     p.fs = 100;

    sim_init();
    {
        sim_t idle = p;
        idle.dc_ir = 2000; idle.dc_red = 1500;
        idle.ac_ir = 0;
        sim_run(&idle, 1500);
    }

    /* Drive it by hand so RED gets independent noise while IR pulses. */
    for (i = 0; i < 2500; i++) {
        double a  = shape_ac(sim_phase);
        double ir = p.dc_ir + a * p.ac_ir;
        double rd = p.dc_red + sim_rand() * 40.0;
        ppg_process((uint32_t)rd, (uint32_t)ir);
        sim_phase += p.bpm / (60.0 * p.fs);
        if (sim_phase >= 1.0) sim_phase -= 1.0;
        g_ms += 10;
        if ((i % 25) == 0) ppg_service();
    }

    CHECK(ppg.finger == 1, "finger not detected");
    CHECK(ppg.bpm_x10 > 600 && ppg.bpm_x10 < 900,
          "heart rate %u not published (want ~720)", ppg.bpm_x10);
    CHECK(ppg.valid == 1,
          "valid not asserted -- heart rate is being withheld because "
          "SpO2 is unavailable");
    CHECK(ppg.spo2_x10 == 0,
          "SpO2 %u published from an uncorrelated red channel",
          ppg.spo2_x10);
    /* And it has to SAY so.  A red channel that is not tracking the pulse
     * is the single most common reason this device declines to measure,
     * and the code that says which reason it was is the only thing that
     * tells it apart from a firmware that is simply broken. */
    CHECK(ppg.spo2_rail == SPO2_CORR,
          "rail=%u on an uncorrelated red channel, want SPO2_CORR=%u",
          ppg.spo2_rail, SPO2_CORR);
}

/* Every way out of the SpO2 update has to leave a reason behind.  Three of
 * them used to return silently, so "no SpO2" covered a dead red emitter, a
 * pulse too weak to form a ratio from and a finger barely on the sensor
 * with the same blank field -- three different faults, three different
 * fixes, and no way to tell them apart on the device. */
static void test_spo2_reason_is_always_given(void)
{
    sim_t p;
    banner("no SpO2 always comes with a reason");

    /* --- a flat red channel: no AC to form a ratio from --- */
    p.bpm = 72.0;
    p.dc_ir = 90000; p.dc_red = 65000;
    p.ac_ir = 900;   p.ac_red = 0;
    p.noise = 0;     p.fs = 100;

    sim_init();
    {
        sim_t idle = p;
        idle.dc_ir = 2000; idle.dc_red = 1500;
        idle.ac_ir = 0;
        sim_run(&idle, 1500);
    }
    sim_run(&p, 20000);
    CHECK(ppg.bpm_x10 > 600 && ppg.bpm_x10 < 900,
          "flat red: heart rate %u not tracked (want ~720)", ppg.bpm_x10);
    CHECK(ppg.spo2_x10 == 0, "flat red: SpO2 %u published", ppg.spo2_x10);
    CHECK(ppg.spo2_rail == SPO2_WEAK,
          "flat red: rail=%u, want SPO2_WEAK=%u", ppg.spo2_rail, SPO2_WEAK);
    /* The perfusion index belongs to the IR channel and must survive a
     * useless red one -- it used to be computed after the red AC test and
     * so was blanked along with the saturation. */
    CHECK(ppg.pi_x100 > 0,
          "flat red: perfusion index blanked by an unusable red channel");
    /* And the two spans have to say WHICH channel is short. */
    CHECK(ppg.fac_ir > 0 && ppg.fac_red < ppg.fac_ir / 4,
          "flat red: AC spans ir=%u red=%u do not identify the weak channel",
          ppg.fac_ir, ppg.fac_red);

    /* --- reversed channels: R arrives as 1/R, off the curve's domain --- */
    p.ac_ir = 400; p.ac_red = 1200;      /* red AC/DC far above IR's */
    sim_init();
    {
        sim_t idle = p;
        idle.dc_ir = 2000; idle.dc_red = 1500;
        idle.ac_ir = 0;    idle.ac_red = 0;
        sim_run(&idle, 1500);
    }
    sim_run(&p, 20000);
    CHECK(ppg.spo2_x10 == 0,
          "R=%.2f: SpO2 %u published from a ratio off the curve",
          ppg.r_q12 / 4096.0, ppg.spo2_x10);
    CHECK(ppg.spo2_rail == SPO2_R_RANGE,
          "reversed pair: rail=%u, want SPO2_R_RANGE=%u",
          ppg.spo2_rail, SPO2_R_RANGE);
}

/* The regression test for the correlation window.
 *
 * A real pulse on a weak red return -- a dark or thick finger, a module
 * whose red emitter couples poorly -- correlates well when measured over
 * several cardiac cycles, as the reference measures it, and erratically
 * when measured over one.  With the sums restarted every beat this signal
 * spent most of a measurement dashed out at rail=SPO2_CORR, reporting that
 * red was not tracking the pulse while red was tracking it perfectly. */
static void test_spo2_weak_red_still_measures(void)
{
    sim_t p;
    double R, want, got;
    banner("SpO2 survives a weak red return");

    p.bpm = 72.0;
    p.dc_ir = 120000; p.dc_red = 14000;   /* red returns a tenth of IR */
    p.ac_ir = 1200;   p.ac_red = 77.0;    /* R about 0.55 all the same */
    p.noise = 40;     p.fs = 100;

    R    = (p.ac_red / p.dc_red) / (p.ac_ir / p.dc_ir);
    want = -45.06 * R * R + 30.354 * R + 94.845;

    sim_init();
    {
        sim_t idle = p;
        idle.dc_ir = 2000; idle.dc_red = 1500;
        idle.ac_ir = 0;    idle.ac_red = 0;
        sim_run(&idle, 1500);
    }
    sim_run(&p, 30000);

    CHECK(ppg.spo2_x10 != 0,
          "weak red (R=%.3f): no SpO2 (rail=%u corr=%u acr=%u)",
          R, ppg.spo2_rail, ppg.corr_x100, ppg.fac_red);
    if (!ppg.spo2_x10) return;
    got = ppg.spo2_x10 / 10.0;
    CHECK(fabs(got - want) < 4.0,
          "weak red: SpO2 %.1f want %.1f (measured R=%.3f, true %.3f)",
          got, want, ppg.r_q12 / 4096.0, R);
    CHECK(ppg.spo2_rail == SPO2_OK,
          "weak red: rail=%u with a reading published", ppg.spo2_rail);
}

static void test_spo2_goes_stale(void)
{
    sim_t p;
    uint16_t published;
    banner("a published SpO2 is retired when it stops being measured");

    p.bpm = 72.0;
    p.dc_ir = 90000; p.dc_red = 65000;
    p.ac_ir = 900;   p.ac_red = 360;
    p.noise = 0;     p.fs = 100;

    sim_init();
    {
        sim_t idle = p;
        idle.dc_ir = 2000; idle.dc_red = 1500;
        idle.ac_ir = 0; idle.ac_red = 0;
        sim_run(&idle, 1500);
    }
    sim_run(&p, 20000);
    published = ppg.spo2_x10;
    CHECK(published != 0, "no SpO2 to go stale");

    /* Keep the pulse on IR -- so beats keep arriving and the heart rate
     * stays valid -- but collapse the red AC so no beat window can yield
     * a ratio any more.  The old code left the last value on display
     * indefinitely in exactly this situation. */
    p.ac_red = 0;
    sim_run(&p, (uint32_t)(SPO2_STALE_MS + 4000));

    CHECK(ppg.valid == 1, "heart rate lost along with SpO2");
    CHECK(ppg.spo2_x10 == 0,
          "stale SpO2 %u still being presented %lu ms after the last "
          "successful ratio", ppg.spo2_x10, (unsigned long)SPO2_STALE_MS);
}

/* ==================================================================
 *  5. Time base: wraparound and lost samples
 * ================================================================== */
static void test_millis_wraparound(void)
{
    sim_t p;
    banner("millis() wraparound");

    p.bpm = 72.0;
    p.dc_ir = 90000; p.dc_red = 65000;
    p.ac_ir = 900;   p.ac_red = 360;
    p.noise = 0;     p.fs = 100;

    /* Start 3 s before the 32-bit millisecond counter wraps and run
     * across it.  Every comparison in the firmware is written as an
     * unsigned difference, so this must be a non-event. */
    sim_init();
    g_ms = 0xFFFFFFFFUL - 3000UL;
    {
        sim_t idle = p;
        idle.dc_ir = 2000; idle.dc_red = 1500;
        idle.ac_ir = 0; idle.ac_red = 0;
        sim_run(&idle, 1500);
    }
    sim_run(&p, 25000);

    CHECK(g_ms < 0x10000000UL, "the clock did not actually wrap (g_ms=%lu)",
          (unsigned long)g_ms);
    CHECK(ppg.finger == 1, "finger lost across the wrap");
    CHECK(ppg.valid == 1, "measurement lost across the wrap");
    CHECK(ppg.bpm_x10 > 650 && ppg.bpm_x10 < 800,
          "rate %u wrong across the wrap (want ~720)", ppg.bpm_x10);
}

static void test_sample_counter_wraparound(void)
{
    sim_t p;
    banner("sample-counter wraparound");

    p.bpm = 72.0;
    p.dc_ir = 90000; p.dc_red = 65000;
    p.ac_ir = 900;   p.ac_red = 360;
    p.noise = 0;     p.fs = 100;

    /* Beats are timed by counting samples, and the crossing timestamp is
     * (nsamp-1)<<8, which overflows a uint32 at 2^24 samples -- about 46
     * hours at 100 Hz.  The interval is a difference of two such stamps,
     * so it stays correct modulo 2^32 provided the gap is short.  Park
     * the counter just under the overflow and check that it is. */
    sim_init();
    {
        sim_t idle = p;
        idle.dc_ir = 2000; idle.dc_red = 1500;
        idle.ac_ir = 0; idle.ac_red = 0;
        sim_run(&idle, 1500);
    }
    sim_run(&p, 12000);
    CHECK(ppg.valid == 1, "did not converge before the wrap");

    s.nsamp = 0x00FFFF80UL;            /* 128 samples from overflow */
    sim_run(&p, 20000);

    CHECK(ppg.valid == 1, "measurement lost across the sample-counter wrap");
    CHECK(ppg.bpm_x10 > 650 && ppg.bpm_x10 < 800,
          "rate %u wrong across the sample-counter wrap (want ~720)",
          ppg.bpm_x10);
}

static void test_lost_samples(void)
{
    banner("lost-sample accounting");

    /* A small loss advances the time base and nothing else: the bus
     * drops a byte often enough that discarding the beat reference for
     * every one starved the median of good intervals. */
    sim_init();
    {
        uint32_t before = s.nsamp;
        uint32_t rej    = ppg.rejects;
        s.t_cross_q8 = 12345;
        ppg_lost_samples(LOST_RESYNC - 1);
        CHECK(s.nsamp == before + (LOST_RESYNC - 1),
              "small loss did not advance the time base");
        CHECK(s.t_cross_q8 == 12345,
              "small loss threw away the beat reference");
        CHECK(ppg.rejects == rej, "small loss counted as a rejection");
    }

    /* A large one is a discontinuity: the reference must go, so no
     * interval is measured across the gap. */
    {
        uint32_t rej = ppg.rejects;
        s.t_cross_q8 = 12345;
        ppg_lost_samples(LOST_RESYNC);
        CHECK(s.t_cross_q8 == 0, "large loss kept a stale beat reference");
        CHECK(ppg.rejects == rej + 1, "large loss not counted");
    }

    /* Zero is a no-op. */
    {
        uint32_t before = s.nsamp;
        ppg_lost_samples(0);
        CHECK(s.nsamp == before, "ppg_lost_samples(0) advanced the counter");
    }

    /* The driver's marker for "the FIFO overflowed but by an unknown
     * amount" has to be big enough to force a resync, or the DSP will
     * happily time an interval straight across the gap. */
    CHECK(MAX_OVF_UNKNOWN >= LOST_RESYNC,
          "MAX_OVF_UNKNOWN=%d is below LOST_RESYNC=%d, so an unquantified "
          "FIFO overflow would not drop the beat reference",
          MAX_OVF_UNKNOWN, LOST_RESYNC);
}

/* ==================================================================
 *  6. Range and overflow safety
 * ================================================================== */
static void test_extremes(void)
{
    sim_t p;
    uint32_t i;
    banner("saturation and extremes");

    /* A railed sensor: both channels pinned at the top of the 18-bit
     * range.  Nothing may be published, and nothing may overflow. */
    sim_init();
    for (i = 0; i < 4000; i++) {
        ppg_process(262143UL, 262143UL);
        g_ms += 10;
        if ((i % 25) == 0) ppg_service();
    }
    CHECK(ppg.spo2_x10 == 0 || (ppg.spo2_x10 >= 700 && ppg.spo2_x10 <= 1000),
          "SpO2 %u out of range on a saturated input", ppg.spo2_x10);
    CHECK(ppg.bpm_x10 == 0 || ppg.bpm_x10 <= 2500,
          "bpm %u out of range on a saturated input", ppg.bpm_x10);
    CHECK(ppg.dc_ir <= 262143UL, "dc_ir %lu above full scale",
          (unsigned long)ppg.dc_ir);

    /* A dead sensor returning zeros: the DC floor exists so nothing
     * downstream divides by it. */
    sim_init();
    for (i = 0; i < 3000; i++) {
        ppg_process(0, 0);
        g_ms += 10;
        if ((i % 25) == 0) ppg_service();
    }
    CHECK(ppg.dc_ir >= 1, "dc_ir fell to 0 -- a divisor downstream");
    CHECK(ppg.dc_red >= 1, "dc_red fell to 0 -- a divisor downstream");
    CHECK(ppg.finger == 0, "finger detected on an all-zero input");

    /* Heavy noise on a real pulse: the published values must stay inside
     * their declared ranges however bad the signal is. */
    p.bpm = 72.0;
    p.dc_ir = 90000; p.dc_red = 65000;
    p.ac_ir = 900;   p.ac_red = 360;
    p.noise = 2500;  p.fs = 100;
    sim_init();
    {
        sim_t idle = p;
        idle.dc_ir = 2000; idle.dc_red = 1500;
        idle.ac_ir = 0; idle.ac_red = 0; idle.noise = 50;
        sim_run(&idle, 1500);
    }
    sim_run(&p, 30000);
    CHECK(ppg.bpm_x10 <= 2500, "bpm_x10 %u above its declared ceiling",
          ppg.bpm_x10);
    CHECK(ppg.spo2_x10 == 0 || (ppg.spo2_x10 >= 700 && ppg.spo2_x10 <= 1000),
          "SpO2 %u outside 70..100 %% under noise", ppg.spo2_x10);
    CHECK(ppg.sqi <= 100, "sqi %u above 100", ppg.sqi);
    CHECK(ppg.progress <= 100, "progress %u above 100", ppg.progress);
    CHECK(ppg.wave >= -110 && ppg.wave <= 110, "wave %d out of range",
          ppg.wave);
    CHECK(ppg.resp_bpm <= 40, "resp %u above its ceiling", ppg.resp_bpm);
}

static void test_waveform_ring(void)
{
    sim_t p;
    uint8_t i;
    int nonzero = 0;
    banner("waveform ring");

    p.bpm = 72.0;
    p.dc_ir = 90000; p.dc_red = 65000;
    p.ac_ir = 900;   p.ac_red = 360;
    p.noise = 0;     p.fs = 100;

    sim_init();
    {
        sim_t idle = p;
        idle.dc_ir = 2000; idle.dc_red = 1500;
        idle.ac_ir = 0; idle.ac_red = 0;
        sim_run(&idle, 1500);
    }
    sim_run(&p, 20000);

    CHECK(ppg_wave_head < WAVE_LEN, "wave head %u out of bounds",
          ppg_wave_head);
    for (i = 0; i < WAVE_LEN; i++) {
        CHECK(ppg_wave[i] >= -110 && ppg_wave[i] <= 110,
              "wave[%u]=%d out of range", i, ppg_wave[i]);
        if (ppg_wave[i]) nonzero++;
    }
    CHECK(nonzero > WAVE_LEN / 2, "only %d of %d wave samples are non-zero",
          nonzero, WAVE_LEN);
}

/* ==================================================================
 *  7. Configuration invariants
 * ================================================================== */
static void test_config_invariants(void)
{
    banner("configuration invariants");

    /* Every selectable averaging setting must be paired with an ADC
     * sample rate that datasheet Table 11 permits at the 411 us pulse
     * width, and must leave the FIFO output rate at 100 Hz.  Table 11
     * allows 50/100/200/400 Hz there and nothing above. */
    {
        uint8_t a;
        for (a = 0; a <= MAX_AVG_MAX; a++) {
            uint16_t adc_hz = (uint16_t)(100u << a);   /* 100 * 2^a */
            uint16_t out_hz = (uint16_t)(adc_hz >> a);
            CHECK(out_hz == 100, "avg %ux gives %u Hz out, want 100",
                  1u << a, out_hz);
            CHECK(adc_hz <= 400,
                  "avg %ux needs a %u Hz ADC rate, which Table 11 does not "
                  "allow at 411 us", 1u << a, adc_hz);
        }
        /* And the next setting up must genuinely be out of reach, or the
         * limit is too conservative. */
        CHECK((uint16_t)(100u << (MAX_AVG_MAX + 1)) > 400,
              "MAX_AVG_MAX is lower than Table 11 requires");
    }

    /* The refractory floor must not be able to blank a real beat at the
     * fastest rate the interval bounds accept. */
    CHECK(REFRAC_MS < IBI_MIN_MS,
          "refractory %d ms is not shorter than the %d ms minimum interval",
          REFRAC_MS, IBI_MIN_MS);
    CHECK(REFRAC_MAX < IBI_MAX_MS, "refractory ceiling exceeds IBI_MAX_MS");

    /* The interval bounds must bracket every rate the display can show. */
    CHECK(600000UL / IBI_MAX_MS >= 20 * 10 - 5, "IBI_MAX_MS too short");
    CHECK(600000UL / IBI_MIN_MS <= 2500, "IBI_MIN_MS admits rates the "
          "display clamps away");

    /* The RMS/correlation accumulators must not be able to overflow.
     * Worst case is RMS_N_MAX samples all clamped to RMS_Q_MAX. */
    {
        double worst = (double)RMS_Q_MAX * RMS_Q_MAX * RMS_N_MAX;
        CHECK(worst <= 4294967295.0,
              "sum of squares can reach %.0f, past a uint32", worst);
        CHECK(worst <= 2147483647.0,
              "cross-product sum can reach %.0f, past an int32", worst);
    }

    /* AC values are clamped to 60000 and then shifted left 16 before the
     * divide, which must stay inside a uint32. */
    CHECK(60000.0 * 65536.0 <= 4294967295.0,
          "the AC/DC ratio overflows a uint32");
}

/* ================================================================== */
int main(void)
{
    printf("PulseOx host tests -- DSP, integer maths and state machine\n");
    shape_calibrate();

    test_isqrt();
    test_median();
    test_spo2_curve();
    test_config_invariants();
    test_beat_rate();
    test_spo2_end_to_end();
    test_no_finger();
    test_finger_release();
    test_hr_without_spo2();
    test_spo2_reason_is_always_given();
    test_spo2_weak_red_still_measures();
    test_spo2_goes_stale();
    test_millis_wraparound();
    test_sample_counter_wraparound();
    test_lost_samples();
    test_extremes();
    test_waveform_ring();

    printf("\n------------------------------------------------------\n");
    printf("%d checks, %d failed\n", g_checks, g_fail);
    if (stub_saves > 4)
        printf("note: %d EEPROM writes during the run (wear check)\n",
               stub_saves);
    return g_fail ? 1 : 0;
}
