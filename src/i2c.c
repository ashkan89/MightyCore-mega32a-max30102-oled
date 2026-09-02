#include "i2c.h"
#include <util/delay.h>
#include <util/twi.h>
#include <avr/wdt.h>

/* One byte plus its ACK is 90 us at 100 kHz and a START is about 10 us, so
 * a couple of milliseconds is already generous -- the MAX30102 does not
 * stretch the clock.  This used to be 40000 spins, roughly 17 ms per stage:
 * a single failed register read then cost 3 retries x 2 stages x 17 ms, over
 * 100 ms, and a handful of them per pass dropped the whole main loop to
 * about 1 Hz.  That starved the FIFO, the beat detector and the status LED
 * all at once, so a bus fault presented as "nothing works" rather than as a
 * bus fault. */
#define TW_TIMEOUT 5000u       /* ~2.2 ms of spin at 16 MHz */

/* A STOP takes about 1.5 SCL periods -- 15 us at 100 kHz.  If it has not
 * happened in a tenth of a millisecond it is not going to, so there is no
 * point spending the full transfer timeout on it. */
#define TW_STOP_SPINS 250u     /* ~110 us */

/* Bus-free time between a STOP and the next START.  The spec minimum is
 * 4.7 us; the AVR TWI does not enforce any, and this part wants a little
 * more than the minimum after a long FIFO burst before it will ACK again. */
#define T_BUF_US      20

static uint16_t s_khz = 100;   /* conservative default: these modules are
                                  routinely marginal at 400 kHz */
static uint8_t  s_status, s_stage;
static uint16_t s_stuck;       /* times the bus had to be force-released */

uint8_t i2c_last_status(void) { return s_status; }
uint8_t i2c_last_stage(void)  { return s_stage;  }
uint16_t i2c_stuck_count(void) { return s_stuck; }

/* TWBR = (F_CPU/SCL - 16) / 2 with the prescaler at 1 */
void i2c_set_speed_khz(uint16_t khz)
{
    s_khz = khz;
    TWSR = 0;
    TWBR = (khz >= 400) ? 12 : 72;         /* 400 kHz : 100 kHz @ 16 MHz */
}

uint16_t i2c_get_speed_khz(void) { return s_khz; }

/* Reads the actual pin levels.  Both should idle HIGH; a line stuck LOW
 * means missing pull-ups, a short, or a slave holding the bus. */
uint8_t i2c_lines(void)
{
    return (uint8_t)(((PINC & (1 << PC0)) ? 1 : 0) |
                     ((PINC & (1 << PC1)) ? 2 : 0));
}

uint8_t i2c_probe(uint8_t addr7)
{
    uint8_t ok = (uint8_t)(i2c_start((uint8_t)(addr7 << 1)) == I2C_OK);
    i2c_stop();
    return ok;
}

uint8_t i2c_scan(uint8_t *found, uint8_t max)
{
    uint8_t a, n = 0;
    for (a = 0x08; a <= 0x77; a++) {
        /* A stuck bus makes every probe burn the full TWI timeout; the scan
         * then outlasts the watchdog unless it is reset as we go. */
        wdt_reset();
        if (i2c_probe(a)) {
            if (found && n < max) found[n] = a;
            n++;
        }
    }
    return n;
}

void i2c_init(void)
{
    /* PC0/PC1 as inputs with pull-ups; external 4k7 pull-ups still required */
    DDRC  &= (uint8_t)~((1 << PC0) | (1 << PC1));
    PORTC |=  (uint8_t)((1 << PC0) | (1 << PC1));

    i2c_set_speed_khz(s_khz);
    TWCR = (1 << TWEN);
}

/* Bit-bang 9 SCL pulses to release a slave that is holding SDA low. */
static void bus_release(void)
{
    uint8_t i;
    TWCR = 0;
    DDRC  |=  (uint8_t)(1 << PC0);         /* SCL output */
    DDRC  &= (uint8_t)~(1 << PC1);         /* SDA input  */
    PORTC |=  (uint8_t)(1 << PC1);
    for (i = 0; i < 9; i++) {
        PORTC &= (uint8_t)~(1 << PC0); _delay_us(5);
        PORTC |=  (uint8_t)(1 << PC0); _delay_us(5);
    }
    /* manual STOP */
    DDRC  |= (uint8_t)(1 << PC1); PORTC &= (uint8_t)~(1 << PC1); _delay_us(5);
    PORTC |= (uint8_t)(1 << PC0);  _delay_us(5);
    DDRC  &= (uint8_t)~(1 << PC1); _delay_us(5);
    i2c_init();
}

void i2c_recover(void) { bus_release(); }

static uint8_t tw_wait(void)
{
    uint16_t t = 0;
    while (!(TWCR & (1 << TWINT)))
        if (++t > TW_TIMEOUT) return I2C_ERR;
    return I2C_OK;
}

uint8_t i2c_start(uint8_t addr_rw)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (tw_wait()) { s_stage = 1; s_status = 0xFF; return I2C_ERR; }
    if ((TWSR & 0xF8) != TW_START && (TWSR & 0xF8) != TW_REP_START) {
        s_stage = 1; s_status = (uint8_t)(TWSR & 0xF8); return I2C_ERR;
    }

    TWDR = addr_rw;
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (tw_wait()) { s_stage = 2; s_status = 0xFF; return I2C_ERR; }
    {
        uint8_t st = (uint8_t)(TWSR & 0xF8);
        if (st != TW_MT_SLA_ACK && st != TW_MR_SLA_ACK) {
            s_stage = 2; s_status = st; return I2C_ERR;
        }
    }
    return I2C_OK;
}

uint8_t i2c_rstart(uint8_t addr_rw) { return i2c_start(addr_rw); }

uint8_t i2c_write(uint8_t d)
{
    TWDR = d;
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (tw_wait()) { s_stage = 3; s_status = 0xFF; return I2C_ERR; }
    if ((TWSR & 0xF8) != TW_MT_DATA_ACK) {
        s_stage = 3; s_status = (uint8_t)(TWSR & 0xF8); return I2C_ERR;
    }
    return I2C_OK;
}

uint8_t i2c_read(uint8_t *d, uint8_t ack)
{
    TWCR = (uint8_t)((1 << TWINT) | (1 << TWEN) | (ack ? (1 << TWEA) : 0));
    if (tw_wait()) { s_stage = 4; s_status = 0xFF; return I2C_ERR; }
    /* The status used to go unchecked, so a byte taken while the TWI had
     * dropped out of master-receive mode -- arbitration lost, a stray START
     * on the line -- was accepted as data.  For the FIFO burst that means
     * silently corrupt samples rather than a reported failure, which is far
     * harder to notice: the DSP just sees nonsense. */
    {
        uint8_t st = (uint8_t)(TWSR & 0xF8);
        uint8_t want = (uint8_t)(ack ? TW_MR_DATA_ACK : TW_MR_DATA_NACK);
        if (st != want) { s_stage = 4; s_status = st; return I2C_ERR; }
    }
    *d = TWDR;
    return I2C_OK;
}

void i2c_stop(void)
{
    uint16_t t = 0;
    uint8_t  in_sr = TWSR;
    uint8_t  st    = (uint8_t)(in_sr & 0xF8);

    /* Abandoning a transaction the slave NACKed is not the same as finishing
     * one.  From the NACK states this part never executes a STOP -- TWSTO
     * just stays set -- so polling for it burned the full timeout and then
     * needed a bus recovery, every time.  Measured on the bench, 100 % of
     * the stuck STOPs entered from TW_MR_SLA_NACK.  Clearing the TWI drops
     * both lines back to the port pull-ups, which the slave reads as an idle
     * bus: immediate, and the correct way out of a NACK. */
    if (st == TW_MT_SLA_NACK || st == TW_MR_SLA_NACK || st == TW_MT_DATA_NACK) {
        TWCR = 0;
        TWCR = (1 << TWEN);
        _delay_us(T_BUF_US);
        return;
    }

    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while ((TWCR & (1 << TWSTO)) && ++t < TW_STOP_SPINS) { }

    if (TWCR & (1 << TWSTO)) {
        s_stuck++;
        /* Leaving TWSTO pending turns a momentary glitch into a permanent
         * fault: with it set the TWI ignores the next TWSTA, so every later
         * transaction times out at the START stage and nothing but a power
         * cycle clears it.  How to clear it depends on why it stuck.  Only
         * bit-bang the bus free if a line really is held down -- doing that
         * unconditionally disables the TWI and re-initialises it mid-stream,
         * which was corrupting the very next transaction. */
        if ((i2c_lines() & 3) != 3) {
            bus_release();
        } else {
            TWCR = 0;
            TWCR = (1 << TWEN);
        }
    }

    /* I2C requires the bus to stay free for at least 4.7 us between a STOP
     * and the next START, and the AVR TWI does not enforce it: write TWSTA as
     * soon as TWSTO clears and the START goes out too early for the slave to
     * see, so it does not ACK its address.  max30102_read() does exactly that
     * -- STOP after writing the FIFO_DATA address, then straight into the
     * burst -- which is why address NACKs arrived at precisely the main-loop
     * rate, once per pass.  Guarding it here covers every call site. */
    _delay_us(T_BUF_US);
}

uint8_t i2c_reg_w8(uint8_t dev, uint8_t reg, uint8_t val)
{
    if (i2c_start((uint8_t)(dev << 1))) goto fail;
    if (i2c_write(reg))                 goto fail;
    if (i2c_write(val))                 goto fail;
    i2c_stop();
    return I2C_OK;
fail:
    i2c_stop();
    return I2C_ERR;
}

uint8_t i2c_reg_rn(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t n)
{
    uint8_t i;
    if (n == 0) return I2C_OK;
    if (i2c_start((uint8_t)(dev << 1)))          goto fail;
    if (i2c_write(reg))                          goto fail;
    if (i2c_rstart((uint8_t)((dev << 1) | 1)))   goto fail;
    for (i = 0; i < n; i++)
        if (i2c_read(&buf[i], (uint8_t)(i < (n - 1)))) goto fail;
    i2c_stop();
    return I2C_OK;
fail:
    i2c_stop();
    return I2C_ERR;
}
