/* max30102.h -- Maxim MAX30102 pulse-oximeter / heart-rate front end */
#ifndef MAX30102_H
#define MAX30102_H
#include "config.h"

#define MAX30102_ADDR   0x57       /* 7-bit */
#define MAX30102_PARTID 0x15

/* register map */
#define REG_INT_ST1     0x00
#define REG_INT_ST2     0x01
#define REG_INT_EN1     0x02
#define REG_INT_EN2     0x03
#define REG_FIFO_WR     0x04
#define REG_FIFO_OVF    0x05
#define REG_FIFO_RD     0x06
#define REG_FIFO_DATA   0x07
#define REG_FIFO_CFG    0x08
#define REG_MODE_CFG    0x09
#define REG_SPO2_CFG    0x0A
#define REG_LED1_PA     0x0C       /* RED */
#define REG_LED2_PA     0x0D       /* IR  */
#define REG_PILOT_PA    0x10
#define REG_MLED_CTRL1  0x11
#define REG_MLED_CTRL2  0x12
#define REG_TEMP_INT    0x1F
#define REG_TEMP_FRAC   0x20
#define REG_TEMP_CFG    0x21
#define REG_REV_ID      0xFE
#define REG_PART_ID     0xFF

/* LED drive current limits used by the AGC (0xFF = 51 mA) */
#define LED_PA_MIN      0x06
/* Ceiling for the AGC.  This was 0x7F, which is ~25 mA per emitter and
 * ~50 mA for the pair while both are pulsing.  Most of these breakouts feed
 * the die from a small on-board LDO, and on a supply that cannot hold up
 * under that the module browns out -- which shows up as bursts of I2C
 * failures and an intermittent SENSOR FAULT, not as anything obviously
 * optical.  0x3F is ~12.6 mA, still double the reference drive below, and a
 * finger has no trouble reaching a usable DC level well inside it. */
#define LED_PA_MAX      0x3F
/* The drive the reference implementation uses (SparkFun setup()'s powerLevel
 * default, and therefore what SunFounder's sketch runs at).  Its 50000-count
 * finger threshold is only meaningful at this current, so it is also what the
 * search-mode gain is pinned to -- see ppg.c. */
#define LED_PA_REF      0x1F

typedef void (*max_sample_cb)(uint32_t red, uint32_t ir);

uint8_t  max30102_init(void);                    /* 1 = ok */
uint8_t  max30102_present(void);
uint8_t  max30102_part_id(void);       /* last value read from reg 0xFF   */
uint8_t  max30102_id_acked(void);      /* 1 = 0x57 ACKed, whatever the ID */
uint8_t  max30102_rev_id(void);
#define MAX_FIFO_ERR 0xFF          /* distinct from any real FIFO depth */
uint8_t  max30102_fifo_count(void);    /* MAX_FIFO_ERR on a bus failure  */
uint16_t max30102_errors(void);        /* consecutive I2C failures       */
uint8_t  max30102_take_ovf(void);      /* samples lost to FIFO overflow  */
uint8_t  max30102_read(max_sample_cb cb, uint8_t max_samples);  /* n consumed */
void     max30102_set_leds(uint8_t red_pa, uint8_t ir_pa);
void     max30102_set_avg(uint8_t avg_code);     /* 0..5 -> 1,2,4,8,16,32 */
void     max30102_shutdown(uint8_t on);
void     max30102_temp_start(void);
uint8_t  max30102_temp_ready(int16_t *temp_x10); /* 1 when a value is fresh  */
void     max30102_flush_fifo(void);

/* Live register read-back, so a stalled sensor can be told apart from a
 * sensor whose configuration never landed over the bus. */
uint8_t  max30102_readback(uint8_t *mode, uint8_t *spo2, uint8_t *fifo);
uint8_t  max30102_ptrs(uint8_t *wr, uint8_t *ovf, uint8_t *rd);

#endif
