/* Host stand-in: program memory is just memory. */
#ifndef HOST_STUB_AVR_PGMSPACE_H
#define HOST_STUB_AVR_PGMSPACE_H
#include <stdint.h>
#define PROGMEM
#define PGM_P const char *
#define PSTR(s) (s)
#define pgm_read_byte(a) (*(const uint8_t *)(a))
#define pgm_read_word(a) (*(const uint16_t *)(a))
#define EEMEM
#endif
