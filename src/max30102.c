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
 *    - the FIFO is tracked by its pointers only; see max30102_fifo_count()
 *      for why the overflow counter cannot be trusted on this module
 * ------------------------------------------------------------------ */
#include "max30102.h"
#include "i2c.h"
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

#define SLOT1_MASK        0xF8
#define SLOT2_MASK        0x8F
#define SLOT_RED_LED      0x01
#define SLOT_IR_LED       0x02

#define I2C_RETRIES       3

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

void max30102_set_avg(uint8_t avg_code)  /* setFIFOAverage */
{
    if (avg_code > 5) avg_code = 5;
    s_avg_code = avg_code;
    write_fifo_cfg();
    max30102_flush_fifo();
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
    if (!reg_read(REG_FIFO_WR,  wr))  return 0;
    if (!reg_read(REG_FIFO_OVF, ovf)) return 0;
    if (!reg_read(REG_FIFO_RD,  rd))  return 0;
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
    bit_mask(REG_SPO2_CFG, SAMPLERATE_MASK, SAMPLERATE_400);
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
    if ((spo2 & 0x1F) != (SAMPLERATE_400 | PULSEWIDTH_411)) return 0;
    return 1;
}

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
/* Pointers only.  FIFO_OVF is deliberately NOT consulted: on this module it
 * reads a saturated 0x1F whether or not anything was dropped, and the same
 * registers come up with FIFO_CFG's A_FULL nibble already set, so the low
 * FIFO registers cannot be trusted as a loss signal.  Believing it cost two
 * separate bugs -- reporting a full 32 samples on an empty FIFO (phantom
 * samples that inflated the sample rate and reset the beat detector), and
 * then flushing the FIFO on every poll so no samples arrived at all.
 * SparkFun ignores the counter too; the pointers are what actually work.
 *
 * The cost of dropping it is that "empty" and "wrapped completely" look
 * alike.  That is a fair trade: the FIFO holds 320 ms at 100 Hz and it is
 * polled every 4 ms, so a full wrap needs an eighty-fold overrun, and the
 * DSP's own interval continuity check rejects the stale beat if it happens. */
uint8_t max30102_fifo_count(void)
{
    uint8_t wr, rd;
    int16_t n;

    if (!reg_read(REG_FIFO_WR, &wr)) return MAX_FIFO_ERR;
    if (!reg_read(REG_FIFO_RD, &rd)) return MAX_FIFO_ERR;

    /* Both pointers are 5-bit fields, so anything above 31 is a corrupt
     * read, not a pointer.  The bus does drop the occasional byte on this
     * board -- roughly one read in a thousand -- and a garbled write pointer
     * produced a bogus count of up to 31, which was reported to the DSP as
     * lost samples and reset the beat detector for no reason. */
    if (wr > 31 || rd > 31) return MAX_FIFO_ERR;

    n = (int16_t)wr - (int16_t)rd;
    if (n < 0) n += 32;

    /* A FIFO found completely full is the one pointer-only sign that time
     * passed without samples reaching the DSP, so the time base is told. */
    if (n >= 31) s_ovf = 31;

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
