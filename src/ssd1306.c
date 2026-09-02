#include "ssd1306.h"
#include <avr/pgmspace.h>
#include <util/delay.h>

uint8_t fb[OLED_W * OLED_H / 8];

#define CS_LO()  (OLED_PORT &= (uint8_t)~(1 << OLED_CS))
#define CS_HI()  (OLED_PORT |=  (uint8_t)(1 << OLED_CS))
#define DC_CMD() (OLED_PORT &= (uint8_t)~(1 << OLED_DC))
#define DC_DAT() (OLED_PORT |=  (uint8_t)(1 << OLED_DC))

void spi_init(void)
{
    OLED_DDR |= (uint8_t)((1 << SPI_MOSI) | (1 << SPI_SCK) |
                          (1 << OLED_CS)  | (1 << OLED_DC) | (1 << OLED_RST));
    OLED_DDR &= (uint8_t)~(1 << SPI_MISO);
#if OLED_CS != SPI_SS
    /* PB4 is the hardware /SS but is not the chip select on this board.  If
     * /SS is an input and goes low, the AVR clears MSTR and drops out of
     * master mode mid-transfer.  A pull-up prevents it from floating low and,
     * unlike driving the pin, cannot contend with anything wired to it.   */
    OLED_DDR  &= (uint8_t)~(1 << SPI_SS);
    OLED_PORT |=  (uint8_t)(1 << SPI_SS);
#endif
    CS_HI();
    /* Master, mode 0, fosc/4 = 4 MHz.  This was fosc/2 = 8 MHz, right at the
     * edge of the SSD1306's ~100 ns write cycle and beyond what these modules
     * hold reliably over jumper wires -- marginal writes land as corrupted
     * pixels, which reads as a flickering panel.  The whole framebuffer still
     * goes out in about 2 ms at 4 MHz, so nothing is lost by halving it. */
    SPCR = (1 << SPE) | (1 << MSTR);
    SPSR = 0;
}

static inline void spi_tx(uint8_t d)
{
    SPDR = d;
    while (!(SPSR & (1 << SPIF))) { }
}

static void cmd(uint8_t c)
{
    CS_LO(); DC_CMD(); spi_tx(c); CS_HI();
}

static void cmd2(uint8_t c, uint8_t a)
{
    CS_LO(); DC_CMD(); spi_tx(c); spi_tx(a); CS_HI();
}

static const uint8_t init_seq[] PROGMEM = {
    0xAE,             /* display off                       */
    0xD5, 0x80,       /* clock div / osc freq              */
    0xA8, 0x3F,       /* multiplex = 64                    */
    0xD3, 0x00,       /* display offset                    */
    0x40,             /* start line 0                      */
    0x8D, 0x14,       /* charge pump on                    */
    0x20, 0x00,       /* horizontal addressing mode        */
    0xA1,             /* segment remap                     */
    0xC8,             /* COM scan direction remapped       */
    0xDA, 0x12,       /* COM pins: alternative, no remap   */
    0x81, 0xCF,       /* contrast                          */
    0xD9, 0xF1,       /* pre-charge                        */
    0xDB, 0x40,       /* VCOMH deselect                    */
    0xA4,             /* resume from RAM                   */
    0xA6,             /* normal (non-inverted)             */
    0x2E,             /* deactivate scroll                 */
    0xAF              /* display on                        */
};

void ssd1306_init(void)
{
    uint8_t i;
    OLED_PORT &= (uint8_t)~(1 << OLED_RST); _delay_ms(20);
    OLED_PORT |=  (uint8_t)(1 << OLED_RST); _delay_ms(20);

    CS_LO(); DC_CMD();
    for (i = 0; i < sizeof(init_seq); i++) spi_tx(pgm_read_byte(&init_seq[i]));
    CS_HI();

    for (i = 0; i < 8; i++) { }
    ssd1306_flush();
}

void ssd1306_contrast(uint8_t c) { cmd2(0x81, c); }
void ssd1306_power(uint8_t on)   { cmd(on ? 0xAF : 0xAE); }

/* The charge pump is the panel's DC-DC.  Switching it off drops the module
 * from milliamps to microamps, which is the whole point of sleeping. */
void ssd1306_charge_pump(uint8_t on) { cmd2(0x8D, (uint8_t)(on ? 0x14 : 0x10)); }

void ssd1306_flip(uint8_t on)
{
    cmd(on ? 0xA0 : 0xA1);
    cmd(on ? 0xC0 : 0xC8);
}

void ssd1306_flush(void)
{
    uint16_t i;
    CS_LO();
    DC_CMD();
    spi_tx(0x21); spi_tx(0); spi_tx(OLED_W - 1);   /* column range */
    spi_tx(0x22); spi_tx(0); spi_tx(7);            /* page range   */
    DC_DAT();
    for (i = 0; i < sizeof(fb); i++) spi_tx(fb[i]);
    CS_HI();
}
