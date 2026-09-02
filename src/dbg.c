#include "dbg.h"

#if DBG_UART

#include "sys.h"
#include "ppg.h"
#include "max30102.h"
#include "i2c.h"
#include <avr/pgmspace.h>
#include <avr/wdt.h>

/* U2X doubles the sampler rate, which halves the divisor error.  At 16 MHz
 * and 38400 that lands within 0.2 %, against 2.1 % for 115200 -- worth the
 * lower rate for a link that has to be trustworthy when nothing else is. */
#define UBRR_U2X ((uint16_t)((F_CPU / (8UL * DBG_BAUD)) - 1UL))

static uint32_t s_loops;

void dbg_loop(void) { s_loops++; }

static void tx(char c)
{
    uint16_t guard = 0;
    while (!(UCSRA & (1 << UDRE)))
        if (++guard > 20000) return;      /* never hang the main loop on it */
    UDR = (uint8_t)c;
}

static void tx_P(const char *s)
{
    char c;
    while ((c = (char)pgm_read_byte(s++)) != '\0') tx(c);
}

static void tx_u32(uint32_t v)
{
    char b[10];
    uint8_t n = 0;
    if (!v) { tx('0'); return; }
    while (v) { b[n++] = (char)('0' + (v % 10)); v /= 10; }
    while (n) tx(b[--n]);
}

/* "name=value" pairs, so the line stays readable as fields come and go */
static void f_hex(const char *name_P, uint8_t v)
{
    static const char h[] PROGMEM = "0123456789ABCDEF";
    tx(' '); tx_P(name_P); tx('=');
    tx((char)pgm_read_byte(&h[v >> 4]));
    tx((char)pgm_read_byte(&h[v & 0x0F]));
}

static void f_dec(const char *name_P, uint32_t v)
{
    tx(' '); tx_P(name_P); tx('='); tx_u32(v);
}

void dbg_init(void)
{
    UBRRH = (uint8_t)(UBRR_U2X >> 8);
    UBRRL = (uint8_t)(UBRR_U2X & 0xFF);
    UCSRA = (1 << U2X);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);   /* 8N1 */
    UCSRB = (1 << TXEN);

#if DBG_MODE == 2
    /* A column header, so a capture opens straight in a spreadsheet
     * without anyone having to remember the field order.  Emitted once,
     * before anything else; every later line is data.  Field meanings are
     * in the README under "Capturing data". */
    tx_P(PSTR("\r\nt_ms,code,ibi_ms,bpm_inst_x10,bpm_x10,amp,"
              "spo2_x10,r_x1000,pi_x100,corr,sqi,fgr,valid,rail,"
              "dc_ir,dc_red,base_ir,ac_ir,ac_red,fac_ir,fac_red,"
              "sps,fs_x100,beats,rej,ovf,i2c_err,stuck,stack_free\r\n"));
#else
    /* The reset cause goes out first, before anything can fail and
     * distract from it.  A board that has been up for days and quietly
     * rebooted is otherwise indistinguishable from one just plugged in --
     * and 'W' here is the difference between "the watchdog is saving us
     * repeatedly" and "nothing is wrong". */
    tx_P(PSTR("\r\n" FW_NAME " " FW_VERSION " diag rst="));
    tx(sys_reset_cause_ch());
    tx_P(PSTR("\r\n"));
#endif
}

/* Ok Short Long Acq Cont Weak -- see the BEAT_* enum in dbg.h */
static const char why[] PROGMEM = "OSLACW";

static char why_ch(uint8_t code)
{
    return (char)pgm_read_byte(&why[(code < 6) ? code : 5]);
}

#if DBG_MODE == 2

/* ",value" -- the separator travels with the value, so a record is one
 * statement per column and the columns cannot drift out of step with the
 * header emitted in dbg_init(). */
static void c_u32(uint32_t v) { tx(','); tx_u32(v); }

/* One CSV record per crossing, accepted or not.
 *
 * Per-crossing rather than per-sample, deliberately.  A per-sample stream
 * of this many fields is about 7 kB/s at the ~120 Hz this board runs,
 * which does not fit in 38400 baud (3.8 kB/s) -- and if the baud rate were
 * raised to carry it, the main loop would spend most of its time blocked
 * inside tx(), perturbing the very sample timing the capture exists to
 * measure.  Two records a second carry every field needed to relate a beat
 * to the signal that produced it, at under 10 % of the link and no
 * measurable effect on FIFO servicing.  README, "Capturing data", covers
 * the raw-sample alternative and what it costs.
 *
 * Rejected crossings are included, with their reason code, because the
 * datasets worth capturing are the ones where beats are NOT being
 * accepted. */
void dbg_beat(uint16_t ibi_ms, uint16_t amp, uint8_t code)
{
    tx_u32(millis());
    tx(',');
    tx(why_ch(code));
    c_u32(ibi_ms);
    /* The rate this one interval implies, next to the median-filtered
     * figure the display shows.  The gap between the two is exactly what
     * the outlier rejection is doing, and it is not visible any other
     * way. */
    c_u32(ibi_ms ? (600000UL / ibi_ms) : 0UL);
    c_u32(ppg.bpm_x10);
    c_u32(amp);
    c_u32(ppg.spo2_x10);
    c_u32(((uint32_t)ppg.r_q12 * 1000UL) >> 12);
    c_u32(ppg.pi_x100);
    c_u32(ppg.corr_x100);
    c_u32(ppg.sqi);
    c_u32(ppg.finger);
    c_u32(ppg.valid);
    c_u32(ppg.spo2_rail);
    c_u32(ppg.dc_ir);
    c_u32(ppg.dc_red);
    c_u32(ppg.base_ir);
    c_u32(ppg.ac_ir);
    c_u32(ppg.ac_red);
    c_u32(ppg.fac_ir);
    c_u32(ppg.fac_red);
    c_u32(ppg.sps);
    c_u32(ppg.fs_x100);
    c_u32(ppg.beats);
    c_u32(ppg.rejects);
    c_u32(max30102_last_ovf());
    c_u32(max30102_errors());
    c_u32(i2c_stuck_count());
    c_u32(sys_stack_free());
    tx_P(PSTR("\r\n"));
}

#else

/* One line per crossing: the interval, the pulse amplitude that produced
 * it, and the reason code.  This is what distinguishes the three failure
 * shapes that all look like "jumping numbers" on the display -- firing
 * twice per beat (alternating short/long), missing beats (intervals at
 * multiples of the true one), and noise (no pattern at all). */
void dbg_beat(uint16_t ibi_ms, uint16_t amp, uint8_t code)
{
    tx_P(PSTR("B ibi="));
    tx_u32(ibi_ms);
    tx_P(PSTR(" amp="));
    tx_u32(amp);
    tx_P(PSTR(" r="));
    tx(why_ch(code));
    tx_P(PSTR("\r\n"));
}

#endif

/* ------------------------------------------------------------------
 *  Channel identification probe
 *
 *  Everything downstream of the FIFO assumes the first 3-byte word of each
 *  sample is RED and the second is IR, which is what the MAX30102 datasheet
 *  specifies for SpO2 mode (0x03) and what the SparkFun reference reads.  If
 *  a part does not honour that -- a relabelled die, clone silicon, anything
 *  whose PART_ID is not 0x15 -- then every AC/DC ratio comes out inverted and
 *  the ratio-of-ratios lands at 1/R.  R then sits above the SpO2 curve's
 *  domain for every real saturation, which reads on the display as an SpO2
 *  that never moves.  No amount of staring at the DSP finds that, because the
 *  DSP is correct; the labels on its inputs are not.
 *
 *  So ask the hardware instead of assuming.  Drive one emitter at a time and
 *  see which word of the pair follows it.  That is a direct, physical answer
 *  and it needs no finger -- the light piped through the module's own plastic
 *  is plenty -- though a finger makes the contrast larger.
 * ------------------------------------------------------------------ */
static uint32_t p_w0, p_w1;
static uint16_t p_n;

static void probe_cb(uint32_t red, uint32_t ir)
{
    /* Named by position, NOT by meaning: w0 is whatever arrives first.
     * Deciding which one is RED is the entire point of the exercise. */
    p_w0 += red;
    p_w1 += ir;
    p_n++;
}

static void probe_wait(uint16_t ms)
{
    uint32_t t = millis();
    while ((uint32_t)(millis() - t) < ms) wdt_reset();
}

/* Returns the two means in *m0 / *m1. */
static void probe_one(const char *label_P, uint8_t pa1, uint8_t pa2,
                      uint32_t *m0, uint32_t *m1)
{
    uint32_t t;

    max30102_set_leds(pa1, pa2);
    probe_wait(150);                  /* let the front end settle ... */
    max30102_flush_fifo();            /* ... then drop what was queued before */

    p_w0 = p_w1 = 0;
    p_n  = 0;
    t = millis();
    while ((uint32_t)(millis() - t) < 400 && p_n < 200) {
        wdt_reset();
        max30102_read(probe_cb, 8);
    }

    *m0 = p_n ? (p_w0 / p_n) : 0;
    *m1 = p_n ? (p_w1 / p_n) : 0;

    tx_P(PSTR("P "));
    tx_P(label_P);
    f_hex(PSTR("pa1"), pa1);
    f_hex(PSTR("pa2"), pa2);
    f_dec(PSTR("n"),    p_n);
    f_dec(PSTR("word0"), *m0);
    f_dec(PSTR("word1"), *m1);
    tx_P(PSTR("\r\n"));
}

uint8_t dbg_channel_probe(void)
{
    uint32_t d0, d1, r0, r1, i0, i1;
    uint8_t  red_is_w0, red_is_w1, ir_is_w1, ir_is_w0, verdict;

    /* Terse on purpose: this build is close to the flash ceiling, and the
     * three data lines above carry the whole answer anyway. */
    tx_P(PSTR("\r\nP probe pa1=LED1(RED) pa2=LED2(IR)\r\n"));

    /* Probe against the datasheet order so probe_cb()'s two arguments really
     * are word0 and word1 by position.  Which of them is RED is the question
     * being asked, so it must not be assumed while asking it. */
    max30102_set_word_order(0);

    probe_one(PSTR("dark"), 0x00,        0x00,        &d0, &d1);
    probe_one(PSTR("red "), LED_PA_MAX,  0x00,        &r0, &r1);
    probe_one(PSTR("ir  "), 0x00,        LED_PA_MAX,  &i0, &i1);

    /* A word has "responded" only if it rose well clear of the dark reading,
     * so ambient drift between the passes cannot decide the verdict. */
    #define ROSE(v, dark)  ((v) > (dark) + 200u + ((dark) >> 2))

    red_is_w0 = (uint8_t)(ROSE(r0, d0) && !ROSE(r1, d1));
    red_is_w1 = (uint8_t)(ROSE(r1, d1) && !ROSE(r0, d0));
    ir_is_w1  = (uint8_t)(ROSE(i1, d1) && !ROSE(i0, d0));
    ir_is_w0  = (uint8_t)(ROSE(i0, d0) && !ROSE(i1, d1));

    /* One token, decoded in the notes above dbg_channel_probe():
     *   OK          word0=RED word1=IR -- the order the driver assumes, so a
     *               stuck SpO2 is something else; read ir/red/aci/acr
     *   REVERSED    word0=IR word1=RED -- every R arrives as 1/R.  Swap the
     *               two words in max30102_read()
     *   NORED/NOIR  that emitter never moved either word: dead or undriven,
     *               which leaves its channel reading noise and inflates R
     *   BOTH        no optical separation; retry with a finger on, out of
     *               strong ambient light */
    tx_P(PSTR("P order="));
    if (red_is_w0 && ir_is_w1) {
        verdict = PROBE_OK;
        tx_P(PSTR("OK"));
    } else if (red_is_w1 && ir_is_w0) {
        /* Act on it rather than just reporting it.  Only a clear verdict gets
         * here: one word had to rise well clear of dark while the other did
         * not, for BOTH emitters, so ambient drift cannot flip the decode.
         * Anything less certain leaves the datasheet order in place. */
        max30102_set_word_order(1);
        verdict = PROBE_REVERSED;
        tx_P(PSTR("REVERSED->swapped"));
    } else if (!ROSE(r0, d0) && !ROSE(r1, d1)) {
        verdict = PROBE_NORED;
        tx_P(PSTR("NORED"));
    } else if (!ROSE(i0, d0) && !ROSE(i1, d1)) {
        verdict = PROBE_NOIR;
        tx_P(PSTR("NOIR"));
    } else {
        verdict = PROBE_BOTH;
        tx_P(PSTR("BOTH"));
    }
    tx_P(PSTR("\r\n"));
    #undef ROSE
    return verdict;
}

#if DBG_MODE == 2 || !DBG_STATUS
/* Nothing periodic here.  In CSV mode a status line every 2 s would land
 * in the middle of the records and stop the capture being a valid CSV
 * file; with DBG_STATUS off it is compiled out to buy flash. */
void dbg_service(void) { }
#else

void dbg_service(void)
{
    static uint32_t next_ms, loop_mark;
    static uint8_t  started;
    uint32_t now = millis();
    uint8_t  mode, spo2, fifo, lines;

    if (started && (int32_t)(now - next_ms) < 0) return;
    started = 1;
    next_ms = now + 2000;

    max30102_readback(&mode, &spo2, &fifo);
    lines = i2c_lines();

    /* --- identity and configuration: these should never change --- */
    f_dec(PSTR("t"),    now / 1000UL);
    f_dec(PSTR("up"),   max30102_present());
    f_hex(PSTR("id"),   max30102_part_id());     /* 15                     */
    f_hex(PSTR("mode"), mode);                   /* 03 = RED+IR            */
    f_hex(PSTR("spo2"), spo2);                   /* 2F = 4096nA/400Hz/411us*/
    f_hex(PSTR("fifo"), fifo);                   /* rollover on, avg 4     */

    /* --- stream health --- */
    f_dec(PSTR("sps"),  ppg.sps);                /* samples/s, want ~100   */
    f_dec(PSTR("fs"),   ppg.fs_x100 / 100UL);
    f_dec(PSTR("lps"),  (s_loops - loop_mark) / 2); /* main-loop passes/s  */
    loop_mark = s_loops;

    /* --- detection --- */
    f_dec(PSTR("ir"),   ppg.dc_ir);
    f_dec(PSTR("refl"), ppg.refl_ir);      /* what the detector compares  */
    f_dec(PSTR("red"),  ppg.dc_red);
    f_dec(PSTR("base"), ppg.base_ir);            /* learned idle level     */
    f_dec(PSTR("th"),   ppg.finger_th);          /* ir has to pass this    */
    f_dec(PSTR("fgr"),  ppg.finger);
    /* Both drives, because the AGC walks them independently: a RED that has
     * railed at the ceiling while IR sits low says the red return is weak,
     * which is one of the ways R comes out too high. */
    f_hex(PSTR("led"),  ppg.led_ir);
    f_hex(PSTR("ledr"), ppg.led_red);

    /* --- measurement --- */
    f_dec(PSTR("bpm"),  ppg.bpm_x10 / 10U);
    f_dec(PSTR("sp"),   ppg.spo2_x10 / 10U);
    f_dec(PSTR("bts"),  ppg.beats);
    f_dec(PSTR("rej"),  ppg.rejects);
    f_dec(PSTR("val"),  ppg.valid);

    /* --- the SpO2 measurement itself ---
     * Everything R is built from, so a wrong SpO2 can be attributed rather
     * than guessed at.  R is printed x1000; the firmware's own numbers must
     * satisfy  r = (acr/red) / (aci/ir) x 1000, and if they do not, the fault
     * is in the arithmetic rather than in the optics.
     *
     *   r  400.. 800   normal.  SpO2 94..99, and sp should agree with it
     *   r 1300..2300   RED and IR are the wrong way round: R has come back
     *                  as 1/R.  Cross-check with pi, which reads about
     *                  R x the true perfusion when the channels are swapped
     *   r  rising with ambient light, movement or a loose finger: common-mode
     *                  interference.  It lands on both channels as the same
     *                  ADC counts, and RED has the smaller DC to divide by,
     *                  so it always pushes R up and SpO2 down
     *   rail=1         R past the curve's domain, so no reading is published
     */
    f_dec(PSTR("r"),    ((uint32_t)ppg.r_q12 * 1000UL) >> 12);
    f_dec(PSTR("aci"),  ppg.fac_ir);         /* band-passed IR  span -> R */
    f_dec(PSTR("acr"),  ppg.fac_red);        /* band-passed RED span -> R */
    f_dec(PSTR("pi"),   ppg.pi_x100);
    /* corr is the red/IR Pearson correlation x100 -- the reference's own
     * quality gate.  It is what separates the two ways R goes wrong:
     *   corr >= 80 with a high r : both channels carry a real, in-step pulse
     *                              but the ratio is inverted -> channel order
     *   corr <  80               : RED is not tracking the pulse at all
     *                              -> dead/unseen emitter, or noise */
    f_dec(PSTR("corr"), ppg.corr_x100);
    f_dec(PSTR("rail"), ppg.spo2_rail);
    f_dec(PSTR("swap"), max30102_ir_first());   /* what the probe decided */

    /* --- bus health: err, stk and a non-3 ln all mean trouble --- */
    f_dec(PSTR("err"),  max30102_errors());
    f_dec(PSTR("stk"),  i2c_stuck_count());
    f_hex(PSTR("tw"),   i2c_last_status());
    f_dec(PSTR("stg"),  i2c_last_stage());
    f_dec(PSTR("ln"),   (uint32_t)(lines & 3));  /* 3 = both idle high     */
    /* OVF_COUNTER exactly as it last read, before the cross-check in
     * max30102_fifo_count() decides whether to believe it.  On a part that
     * behaves as the datasheet describes this is 0 except just after a real
     * overflow.  If it instead sits at 31 while sps is healthy and the FIFO
     * is draining normally, this board is the one the old driver comment
     * described and the cross-check is what is keeping the sample time base
     * honest -- worth knowing, and it costs two characters. */
    f_dec(PSTR("ovf"),  max30102_last_ovf());

    /* --- health of the firmware itself, rather than of the sensor --- */
    /* Bytes of stack paint still standing: the real margin between the
     * stack and .bss.  A static estimate cannot see interrupt frames or
     * what the optimiser did to them, so this is the only trustworthy
     * figure.  It should settle within the first minute and then never
     * move; one that keeps falling over hours is something recursing or
     * leaking, and 0 means the stack has already reached .bss and the
     * corruption has happened. */
    f_dec(PSTR("stack"), sys_stack_free());
    /* Why the board last restarted: P power-on, E external, B brown-out,
     * W watchdog.  Without it a board that has been up for days and
     * quietly rebooted looks exactly like one just plugged in. */
    tx_P(PSTR(" rst="));
    tx(sys_reset_cause_ch());
    tx_P(PSTR("\r\n"));
}
#endif

#endif /* DBG_UART */
