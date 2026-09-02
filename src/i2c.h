/* i2c.h -- hardware TWI master for ATmega32 (SCL=PC0, SDA=PC1), 400 kHz */
#ifndef I2C_H
#define I2C_H
#include "config.h"

#define I2C_OK   0
#define I2C_ERR  1

void    i2c_init(void);
void     i2c_set_speed_khz(uint16_t khz);  /* 100 or 400                   */
uint16_t i2c_get_speed_khz(void);
uint8_t  i2c_probe(uint8_t addr7);         /* 1 = device ACKed             */
uint8_t  i2c_scan(uint8_t *found, uint8_t max);   /* returns device count  */
uint8_t  i2c_lines(void);                  /* bit0 = SCL level, bit1 = SDA */
uint8_t  i2c_last_status(void);            /* TWSR of the last failure     */
uint8_t  i2c_last_stage(void);             /* 1=START 2=ADDR 3=WRITE 4=READ */
uint16_t i2c_stuck_count(void);            /* times the bus was force-freed */
void    i2c_recover(void);                 /* frees a stuck bus (9 clocks) */

uint8_t i2c_start(uint8_t addr_rw);        /* addr already shifted + R/W   */
uint8_t i2c_rstart(uint8_t addr_rw);
uint8_t i2c_write(uint8_t d);
uint8_t i2c_read(uint8_t *d, uint8_t ack);
void    i2c_stop(void);

uint8_t i2c_reg_w8(uint8_t dev, uint8_t reg, uint8_t val);
uint8_t i2c_reg_rn(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t n);

#endif
