/* ------------------------------------------------------------------
 *  max30102.c -- driver ported faithfully from the SparkFun MAX3010x
 *                library (MAX30105.cpp), the de-facto reference
 *                implementation for this part.
 *
 *  Structure, register masks, configuration order and the FIFO read
 *  sequence all mirror that library.  Deliberate additions for a
 *  bare-metal target, which Arduino's Wire hides from you:
 *
 *    - every transaction reports success/failure and is retried
 *    - failures are counted and escalated (recover, slow down, give up)
 *    - the FIFO pointers and the overflow counter are read in a single
 *      burst and cross-checked against each other; see
 *      max30102_fifo_count()
 * ------------------------------------------------------------------ */
#include "max30102.h"
#include "i2c.h"
#include "sys.h"
#include <util/delay.h>
#include <avr/wdt.h>

/* ---- register masks, values as defined by the SparkFun library ---- */
#define ROLLOVER_ENABLE   0x10

#define SHUTDOWN_MASK     0x7F
#define SHUTDOWN_BIT      0x80
#define RESET_MASK        0xBF
#define RESET_BIT         0x40
#define MODE_MASK         0xF8
#define MODE_REDIRONLY    0x03

#define ADCRANGE_MASK     0x9F
#define ADCRANGE_4096     0x20
#define SAMPLERATE_MASK   0xE3
#define SAMPLERATE_400    0x0C
#define PULSEWIDTH_MASK   0xFC
#define PULSEWIDTH_411    0x03

/* SPO2_SR[2:0] lives in bits 4:2 of SPO2_CFG.  Datasheet Table 6:
 *   001 = 100 Hz   010 = 200 Hz   011 = 400 Hz
 * so the field VALUE for the rate we want is (avg_code + 1) and it is
 * placed by shifting left 2.  See sr_field_for_avg(). */
#define SR_FIELD(v)       (uint8_t)((v) << 2)

#define SLOT1_MASK        0xF8
#define SLOT2_MASK        0x8F
#define SLOT_RED_LED      0x01
#define SLOT_IR_LED       0x02

#define I2C_RETRIES       3

/* millis() of the last moment the part's OVF_COUNTER is known to have
 * been cleared -- a successful FIFO drain, or a pointer flush.  See the
 * long note above max30102_fifo_count() for what it is used for. */
static uint32_t s_drain_ms;

static uint8_t  s_part, s_rev, s_present, s_acked;
static uint16_t s_err;
static uint8_t  s_ovf;
static uint8_t  s_avg_code = 2;

/* Which 3-byte word of each FIFO sample is which emitter.  0 = the datasheet's
 * SpO2-mode order, RED first then IR, which is what every genuine MAX30102
 * does and what this driver defaults to.  Parts that disagree -- relabelled
 * dies, clone silicon, anything whose PART_ID is not 0x15 -- hand back the
 * pair the other way round, and because the DSP divides one channel by the
 * other, that inverts the ratio-of-ratios into 1/R.  R then sits permanently
 * off the end of the SpO2 curve, which is indistinguishable from the sensor
 * simply refusing to read.  dbg_channel_probe() settles it against the
 * hardware at start-up and sets this. */
static uint8_t  s_ir_first;

void max30102_set_word_order(uint8_t ir_first) { s_ir_first = ir_first ? 1 : 0; }
uint8_t max30102_ir_first(void) { return s_ir_first; }

uint8_t  max30102_present(void)  { return s_present; }
uint8_t  max30102_part_id(void)  { return s_part;    }
uint8_t  max30102_rev_id(void)   { return s_rev;     }
uint8_t  max30102_id_acked(void) { return s_acked;   }
uint16_t max30102_errors(void)   { return s_err;     }
uint8_t  max30102_take_ovf(void) { uint8_t v = s_ovf; s_ovf = 0; return v; }

/* A dead bus used to be indistinguishable from an empty FIFO, so the firmware
 * would sit there reporting a healthy but idle sensor forever. */
static void bus_fail(void)
{
    s_err++;
    if (s_err == 20) {
        i2c_recover();
        if (i2c_get_speed_khz() > 100) { i2c_set_speed_khz(100); i2c_init(); }
    } else if (s_err >= 150) {
        s_err = 0;
        s_present = 0;
    }
}

/* ---------------- register access (SparkFun semantics) ---------------- */

/* readRegister8: address write, REPEATED START, one byte read. */
static uint8_t reg_read(uint8_t reg, uint8_t *val)
{
    uint8_t t;
    for (t = 0; t < I2C_RETRIES; t++) {
        if (i2c_reg_rn(MAX30102_ADDR, reg, val, 1) == I2C_OK) return 1;
        /* A failing transaction burns the full TWI timeout, and bring-up does
         * dozens of them back to back.  Without this the 2 s watchdog fires
         * part way through init and the board boot-loops instead of ever
         * reaching the fault screen. */
        wdt_reset();
        _delay_us(200);
    }
    return 0;
}

/* Same, for a run of consecutive registers in ONE transaction.
 *
 * The datasheet is explicit that this works: "When reading the MAX30102
 * registers in one burst-read I2C transaction, the register address
 * pointer typically increments so that the next byte of data sent is from
 * the next register... The exception to this is the FIFO data register,
 * register 0x07."  So every register except 0x07 can be swept in one go.
 */
static uint8_t reg_read_n(uint8_t reg, uint8_t *buf, uint8_t n)
{
    uint8_t t;
    for (t = 0; t < I2C_RETRIES; t++) {
        if (i2c_reg_rn(MAX30102_ADDR, reg, buf, n) == I2C_OK) return 1;
        wdt_reset();
        _delay_us(200);
    }
    return 0;
}

/* writeRegister8: address + value, terminated with a STOP. */
static uint8_t reg_write(uint8_t reg, uint8_t val)
{
    uint8_t t;
    for (t = 0; t < I2C_RETRIES; t++) {
        if (i2c_reg_w8(MAX30102_ADDR, reg, val) == I2C_OK) return 1;
        wdt_reset();
        _delay_us(200);
    }
    return 0;
}

/* bitMask: read-modify-write, so reserved bits are preserved rather than
 * being blown away by a whole-register write. */
static uint8_t bit_mask(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t v;
    if (!reg_read(reg, &v)) return 0;
    v = (uint8_t)((v & mask) | value);
    return reg_write(reg, v);
}

/* ---------------- public configuration helpers ---------------- */
void max30102_set_leds(uint8_t red_pa, uint8_t ir_pa)
{
    reg_write(REG_LED1_PA, red_pa);      /* setPulseAmplitudeRed */
    reg_write(REG_LED2_PA, ir_pa);       /* setPulseAmplitudeIR  */
}

void max30102_flush_fifo(void)           /* clearFIFO */
{
    reg_write(REG_FIFO_WR,  0x00);
    reg_write(REG_FIFO_OVF, 0x00);
    reg_write(REG_FIFO_RD,  0x00);
    /* The overflow counter has just been zeroed by hand, so the same
     * reasoning applies as after a drain: anything reported from here is
     * loss that happened afterwards.  Without this, a flush during
     * bring-up left the gate open on a stale count. */
    s_drain_ms = millis();
}

/* FIFO_CFG written whole rather than read-modify-write.  The reference uses
 * two bitMask() calls, which preserve A_FULL -- and this part comes up with
 * A_FULL already set to 0x0F, so the register read back as 0x5F instead of
 * 0x50 and made a correctly configured chip look wrong on the fault screen.
 * Every bit in this register is ours, so just state all of them. */
static uint8_t write_fifo_cfg(void)
{
    return reg_write(REG_FIFO_CFG,
                     (uint8_t)((s_avg_code << 5) | ROLLOVER_ENABLE));
}

/* ---- averaging, and why it also writes the sample rate ----
 *
 * SMP_AVE averages adjacent ADC samples on the chip and pushes one
 * result into the FIFO, so it DIVIDES the FIFO output rate: at the
 * 400 Hz ADC rate this driver configures, 1x averaging gives 400 Hz out
 * and 32x gives 12.5 Hz.  This function used to write SMP_AVE alone,
 * which made the Averaging menu a hidden sample-rate control spanning a
 * factor of 32 -- and every DSP filter corner in ppg.c is a fixed shift
 * that is only correct at one rate, so selecting anything but the default
 * silently moved the passband off the heart rate.  See the notes above
 * DC_SHIFT in ppg.c for what that looked like.
 *
 * The fix is to compensate with SPO2_SR so the output rate is 100 Hz
 * whatever the averaging is:
 *
 *     avg 1x  ->  SPO2_SR = 100 Hz   ->  100 / 1  = 100 Hz out
 *     avg 2x  ->  SPO2_SR = 200 Hz   ->  200 / 2  = 100 Hz out
 *     avg 4x  ->  SPO2_SR = 400 Hz   ->  400 / 4  = 100 Hz out
 *
 * which is why MAX_AVG_MAX is 2 (4x) and not 5 (32x): 8x averaging would
 * need an 800 Hz ADC rate, and datasheet Table 11, "SpO2 Mode (Allowed
 * Settings)", does not permit 800 Hz at the 411 us pulse width this
 * driver uses -- 400 Hz is the highest allowed there.  Writing 800
 * anyway would not fail cleanly either: the datasheet says the part
 * silently programs "the highest possible sample rate" instead, so the
 * register would read back 400 and the output rate would halve without
 * anything reporting it.
 *
 * The averaging setting therefore now means only what its name says --
 * how much on-chip noise averaging is applied -- and trades resolution
 * against nothing else. */
void max30102_set_avg(uint8_t avg_code)  /* setFIFOAverage + matching SR */
{
    if (avg_code > MAX_AVG_MAX) avg_code = MAX_AVG_MAX;
    s_avg_code = avg_code;
    write_fifo_cfg();
    bit_mask(REG_SPO2_CFG, SAMPLERATE_MASK, SR_FIELD(avg_code + 1));
    max30102_flush_fifo();
}

/* The SPO2_CFG sample-rate field the current averaging setting implies. */
static uint8_t sr_field_for_avg(void)
{
    return SR_FIELD(s_avg_code + 1);
}

void max30102_shutdown(uint8_t on)
{
    bit_mask(REG_MODE_CFG, SHUTDOWN_MASK, (uint8_t)(on ? SHUTDOWN_BIT : 0x00));
}

uint8_t max30102_readback(uint8_t *mode, uint8_t *spo2, uint8_t *fifo)
{
    uint8_t ok = 1;
    if (!reg_read(REG_MODE_CFG, mode)) { *mode = 0xEE; ok = 0; }
    if (!reg_read(REG_SPO2_CFG, spo2)) { *spo2 = 0xEE; ok = 0; }
    if (!reg_read(REG_FIFO_CFG, fifo)) { *fifo = 0xEE; ok = 0; }
    return ok;
}

uint8_t max30102_ptrs(uint8_t *wr, uint8_t *ovf, uint8_t *rd)
{
    uint8_t p[3];
    if (!reg_read_n(REG_FIFO_WR, p, 3)) return 0;
    *wr = p[0]; *ovf = p[1]; *rd = p[2];
    return 1;
}

/* ---------------- softReset ---------------- */
/* Best effort, exactly like the reference: SparkFun's softReset() polls for
 * 100 ms and then carries on regardless of what the register says.  This used
 * to return a status that aborted init, which turned a slow-to-clear reset bit
 * -- or a single dropped read during the reset window, when the part is
 * legitimately unresponsive -- into a permanent SENSOR FAULT.  Nothing is
 * lost by continuing: setup_config() below writes every register we rely on. */
static void soft_reset(void)
{
    uint8_t i, v = RESET_BIT;

    bit_mask(REG_MODE_CFG, RESET_MASK, RESET_BIT);
    for (i = 0; i < 100; i++) {            /* SparkFun polls for 100 ms */
        wdt_reset();
        _delay_ms(1);
        if (!reg_read(REG_MODE_CFG, &v)) continue;
        if ((v & RESET_BIT) == 0) return;
    }
    /* Still set (or unreadable): clear the whole mode register by hand so we
     * are not left in shutdown with the reset bit latched. */
    reg_write(REG_MODE_CFG, 0x00);
}

/* ---------------- setup() ---------------- */
static uint8_t read_part_id(void)
{
    s_acked = reg_read(REG_PART_ID, &s_part);
    return s_acked;
}

/* SparkFun setup() order, verbatim.  Kept in one function so a failed
 * read-back can simply run it again. */
static void setup_config(void)
{
    /* interrupts are unused: we poll, exactly as the SparkFun example does */
    reg_write(REG_INT_EN1, 0x00);
    reg_write(REG_INT_EN2, 0x00);
    wdt_reset();

    write_fifo_cfg();
    bit_mask(REG_MODE_CFG, MODE_MASK, MODE_REDIRONLY);      /* RED + IR   */
    bit_mask(REG_SPO2_CFG, ADCRANGE_MASK,   ADCRANGE_4096);
    /* Not a hard-coded 400 Hz: the rate has to match the averaging
     * setting so the FIFO output rate stays at 100 Hz.  See
     * max30102_set_avg().  With the default 4x averaging this is exactly
     * SAMPLERATE_400, so nothing changes for the default build. */
    bit_mask(REG_SPO2_CFG, SAMPLERATE_MASK, sr_field_for_avg());
    bit_mask(REG_SPO2_CFG, PULSEWIDTH_MASK, PULSEWIDTH_411);
    wdt_reset();

    /* Start at the reference drive rather than 0x28: the 50000-count finger
     * threshold the reference uses is calibrated to this current, and ppg.c
     * pins the search gain here for the same reason. */
    max30102_set_leds(LED_PA_REF, LED_PA_REF);
    reg_write(REG_PILOT_PA, LED_PA_REF);

    /* enableSlot(1, RED), enableSlot(2, IR).  SparkFun writes these even in
     * two-LED mode, so we do too rather than assuming reset defaults. */
    bit_mask(REG_MLED_CTRL1, SLOT1_MASK, SLOT_RED_LED);
    bit_mask(REG_MLED_CTRL1, SLOT2_MASK, (uint8_t)(SLOT_IR_LED << 4));

    max30102_flush_fifo();
    wdt_reset();
}

/* Did the configuration actually land?  Checks the two registers the sample
 * format depends on, so a sensor that ACKs but never got configured is told
 * apart from one that is genuinely streaming. */
static uint8_t config_landed(void)
{
    uint8_t mode = 0, spo2 = 0;
    if (!reg_read(REG_MODE_CFG, &mode)) return 0;
    if (!reg_read(REG_SPO2_CFG, &spo2)) return 0;
    if ((mode & 0x07) != MODE_REDIRONLY) return 0;
    if ((spo2 & 0x1F) != (uint8_t)(sr_field_for_avg() | PULSEWIDTH_411)) return 0;
    return 1;
}

/* Public wrapper, so the display can render a verdict instead of a
 * hard-coded list of expected register values.  The monitor screen used to
 * print "want 03 2F 50" next to the live read-back, which stopped being
 * true the moment SPO2_CFG's sample-rate field started tracking the
 * averaging setting -- 4x averaging still wants 0x2F, but 2x wants 0x2B
 * and 1x wants 0x27.  Asking the driver is correct at every setting, and
 * smaller than the string it replaces. */
uint8_t max30102_config_ok(void) { return config_landed(); }

uint8_t max30102_init(void)
{
    uint8_t attempt;

    s_present = 0;
    s_part    = 0;
    s_err     = 0;
    s_ovf     = 0;

    /* The module's LDO and internal oscillator need a moment after power is
     * applied, and on a cold boot we get here well inside that window --
     * the first PART_ID read would NACK and take the whole escalation path
     * with it. */
    _delay_ms(50);
    wdt_reset();

    if (!read_part_id()) {
        i2c_recover();
        if (!read_part_id()) {
            i2c_set_speed_khz(100);
            i2c_init();
            if (!read_part_id()) {
                i2c_recover();
                if (!read_part_id()) return 0;
            }
        }
    }

    /* A PART_ID other than 0x15 used to abort here.  Boards sold as MAX30102
     * routinely answer with something else -- relabelled parts, MAX30105 dies,
     * clone silicon -- while driving the same register map, so configure it
     * anyway and let config_landed() decide.  s_part is still reported, so the
     * fault screen shows what the part actually said. */
    reg_read(REG_REV_ID, &s_rev);

    /* Two goes: one dropped write during the first pass is not a dead sensor,
     * and the read-back is what makes retrying safe. */
    for (attempt = 0; attempt < 2; attempt++) {
        wdt_reset();
        soft_reset();
        setup_config();
        if (config_landed()) {
            s_present = 1;
            return 1;
        }
    }
    return 0;
}

/* ---------------- check() ---------------- */
/* How many unread samples the FIFO holds, and how many it dropped.
 *
 * Both come back in ONE I2C transaction now.  FIFO_WR_PTR (0x04),
 * OVF_COUNTER (0x05) and FIFO_RD_PTR (0x06) are consecutive, and the
 * datasheet guarantees the register address pointer auto-increments
 * across a burst read of anything but 0x07 -- so a 3-byte read replaces
 * the two separate single-register reads this used to do.
 *
 * That halves the transaction count on the hottest path in the firmware.
 * Each register read is an address phase plus a repeated START plus a
 * data byte, about 600 us at 100 kHz; two of them every FIFO_POLL_MS = 4
 * ms was around 30 % of all CPU time spent holding the bus, for nothing
 * but a sample count.  One 3-byte read is about 750 us, so the poll cost
 * drops by roughly 40 % and the FIFO service latency drops with it.
 *
 * ---- the overflow counter ----
 * This code used to ignore OVF_COUNTER entirely and infer loss from the
 * pointers alone, on the bench observation that the counter read a
 * saturated 0x1F whether or not anything had been dropped.  The datasheet
 * says otherwise, and specifically: "When a complete sample is popped
 * (i.e. removal of old FIFO data and shifting the samples down) from the
 * FIFO (when the read pointer advances), OVF_COUNTER is reset to zero."
 * Since this driver pops samples on nearly every poll, the counter read
 * here is the loss accumulated since the previous burst -- exactly the
 * number the DSP's time base needs, and better than the previous guess of
 * "call it 31".
 *
 * Rather than pick a side, the value is cross-checked before it is
 * believed -- against ELAPSED TIME, not against the pointers.  The
 * pointers cannot do this job: with FIFO_ROLLOVER_EN set the write
 * pointer wraps straight past the read pointer, so after a real overflow
 * the difference between them is (32 + lost) mod 32, which is small for a
 * small loss and arbitrary for a large one.  A "the FIFO looks full"
 * test would therefore reject precisely the genuine overflows it was
 * meant to confirm.
 *
 * Time is the honest discriminator.  Samples can only be lost once the
 * FIFO has filled, and the FIFO holds 32 samples -- 320 ms at the 100 Hz
 * output rate.  So a loss reported less than OVF_MIN_GAP_MS after the
 * last successful drain cannot be describing this bus, and is the stuck
 * counter the old comment recorded.  A loss reported after a real stall
 * is believed.  Both worlds are handled, and s_ovf_raw publishes the
 * unfiltered value so which one this board lives in can be read off the
 * diagnostic line within seconds.
 *
 * The pointer-only fallback is kept for the case where the FIFO is found
 * completely full and the counter reported nothing: that is still a sign
 * that time passed without samples reaching the DSP.
 */
/* 200 ms, comfortably inside the 320 ms the FIFO buffers, so a genuine
 * overflow is never rejected, while the every-4-ms polling of a healthy
 * bus can never clear it. */
#define OVF_MIN_GAP_MS 200

static uint8_t  s_ovf_raw;         /* OVF_COUNTER exactly as it read */

uint8_t max30102_last_ovf(void) { return s_ovf_raw; }

uint8_t max30102_fifo_count(void)
{
    uint8_t p[3];
    uint8_t wr, ovf, rd;
    int16_t n;

    if (!reg_read_n(REG_FIFO_WR, p, 3)) return MAX_FIFO_ERR;
    wr  = p[0];
    ovf = p[1];
    rd  = p[2];

    /* All three are 5-bit fields, so anything above 31 is a corrupt read,
     * not a pointer.  The bus does drop the occasional byte on this board
     * -- roughly one read in a thousand -- and a garbled write pointer
     * produced a bogus count of up to 31, which was reported to the DSP
     * as lost samples and reset the beat detector for no reason. */
    if (wr > 31 || rd > 31 || ovf > 31) return MAX_FIFO_ERR;

    s_ovf_raw = ovf;

    n = (int16_t)wr - (int16_t)rd;
    if (n < 0) n += 32;

    if (ovf && (uint32_t)(millis() - s_drain_ms) >= OVF_MIN_GAP_MS) {
        /* Believed: enough time passed un-drained for the FIFO to fill.
         * 31 is the counter's saturation point and means "at least 31,
         * actual number unknown", so it is reported as the marker rather
         * than as a count. */
        s_ovf = (uint8_t)((ovf >= 31) ? MAX_OVF_UNKNOWN : ovf);
    } else if (n >= 31) {
        /* The counter said nothing usable, but the FIFO is full, so
         * samples were still lost.  How many is unknown -- report the
         * discontinuity, not a made-up count.  See MAX_OVF_UNKNOWN. */
        s_ovf = MAX_OVF_UNKNOWN;
    }

    return (uint8_t)n;
}

/* SparkFun's check() writes the FIFO_DATA address and terminates that
 * transaction with a STOP, then issues a fresh START to read the burst.
 * Our previous implementation used a repeated START here, which is the one
 * place the two differ -- so follow the reference. */
/* MAX_BURST samples of RED+IR, 3 bytes each.  The burst is buffered rather
 * than handed to the callback byte-group by byte-group, because the callback
 * is the whole DSP: a median sort, an integer square root and several 32-bit
 * divides.  Running that between two reads of an open burst holds SCL low for
 * the duration -- legal clock stretching, but it kept the bus occupied for
 * milliseconds at a time and left a long window in which any disturbance
 * aborted the transfer.  48 bytes of stack buys a burst that is over in
 * roughly a millisecond. */
#define MAX_BURST 8

uint8_t max30102_read(max_sample_cb cb, uint8_t max_samples)
{
    uint8_t b[MAX_BURST * 6], i, k, n;

    n = max30102_fifo_count();
    if (n == MAX_FIFO_ERR) { bus_fail(); return 0; }
    s_err = 0;
    if (!n) return 0;
    if (n > max_samples) n = max_samples;
    if (n > MAX_BURST)   n = MAX_BURST;

    /* Address phase and burst joined by a REPEATED START, which is what the
     * MAX30102 datasheet's own FIFO-read pseudocode does -- and what every
     * other read here already does via i2c_reg_rn().
     *
     * This used to close the address phase with a STOP and open the burst
     * with a fresh START, because SparkFun's check() calls endTransmission()
     * with no argument.  Arduino's Wire buffers the whole transaction and
     * re-addresses inside requestFrom(), so that works out there; sequenced
     * directly on the TWI it does not.  The part NACKed SLA+R every single
     * time -- once per main-loop pass, which is why the failures arrived at
     * exactly the loop rate -- and the STOP after the NACK then would not
     * complete, so the bus had to be force-freed 40-odd times a second.
     * That, not the optics and not the thresholds, is what stopped the FIFO
     * from streaming and the finger from ever being detected. */
    if (i2c_start((uint8_t)(MAX30102_ADDR << 1))) { i2c_stop(); bus_fail(); return 0; }
    if (i2c_write(REG_FIFO_DATA))                 { i2c_stop(); bus_fail(); return 0; }
    if (i2c_rstart((uint8_t)((MAX30102_ADDR << 1) | 1))) goto fail;

    k = (uint8_t)(n * 6);
    for (i = 0; i < k; i++)
        if (i2c_read(&b[i], (uint8_t)(i < (k - 1)))) goto fail;
    i2c_stop();

    /* The read pointer has advanced, so the part has cleared
     * OVF_COUNTER; anything it reports from here is loss that happened
     * after this moment.  See the note above max30102_fifo_count(). */
    s_drain_ms = millis();

    for (i = 0; i < n; i++) {
        const uint8_t *p = &b[i * 6];
        uint32_t w0 = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
        uint32_t w1 = ((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 8) | p[5];
        w0 &= 0x03FFFFUL;                      /* 18-bit, as tempLong & 0x3FFFF */
        w1 &= 0x03FFFFUL;
        /* Which word is which is settled by dbg_channel_probe() against the
         * hardware, not assumed from the datasheet -- see s_ir_first. */
        if (cb) { if (s_ir_first) cb(w1, w0); else cb(w0, w1); }
    }
    return n;

fail:
    i2c_stop();
    bus_fail();
    /* A burst that died part way through a sample leaves the part's internal
     * byte position out of step with its read pointer, so every later sample
     * comes back with RED and IR split across the wrong boundary -- garbage
     * DC, and finger detection that can never trigger.  Realign by resetting
     * the pointers; a few lost samples are much cheaper than a stream that
     * silently stays wrong until the next power cycle.
     *
     * The samples the flush discards really are gone, so report them: the
     * DSP times beats by counting samples and would otherwise measure the
     * interval spanning the gap as though no time had passed. */
    max30102_flush_fifo();
    s_ovf = n;
    return 0;
}

/* Marker used when the FIFO is known to have overflowed but the number of
 * samples lost is not known -- see the fallback in max30102_fifo_count().
 * It is deliberately small.  Reporting 31 there, as this used to, told the
 * DSP that a third of a second had passed unobserved every time the FIFO
 * was found full, and s.nsamp is also the basis of the sample-rate
 * calibration, so each event skewed the measured rate as well.  What the
 * DSP actually needs from an unquantified gap is the DISCONTINUITY -- drop
 * the beat reference, do not time an interval across it -- and any value
 * at or above LOST_RESYNC achieves that without inflating the time base.  */

/* ---------------- die temperature ---------------- */
static uint8_t s_temp_busy;

void max30102_temp_start(void)
{
    if (s_temp_busy) return;
    if (reg_write(REG_TEMP_CFG, 0x01)) s_temp_busy = 1;
}

uint8_t max30102_temp_ready(int16_t *temp_x10)
{
    uint8_t cfg = 1, ti, tf;
    if (!s_temp_busy) return 0;
    if (!reg_read(REG_TEMP_CFG, &cfg)) { s_temp_busy = 0; return 0; }
    if (cfg & 0x01) return 0;                  /* conversion still running */
    s_temp_busy = 0;
    if (!reg_read(REG_TEMP_INT,  &ti)) return 0;
    if (!reg_read(REG_TEMP_FRAC, &tf)) return 0;
    /* integer part is signed C, fraction is 0.0625 C per LSB */
    *temp_x10 = (int16_t)((int16_t)(int8_t)ti * 10 + (int16_t)((tf & 0x0F) * 10) / 16);
    return 1;
}
