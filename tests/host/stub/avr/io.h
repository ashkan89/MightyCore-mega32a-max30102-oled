/* ------------------------------------------------------------------
 *  tests/host/stub/avr/io.h -- host stand-in for the AVR header
 *
 *  Just enough of avr/io.h to let the pure logic in ppg.c and sys.c be
 *  compiled and exercised by a host compiler.  Every "register" is an
 *  ordinary variable, so code that touches one still compiles and still
 *  reads back what it wrote; nothing here pretends to model the
 *  peripheral behind it.  The tests never depend on peripheral effects
 *  -- they drive the DSP through its function interface.
 * ------------------------------------------------------------------ */
#ifndef HOST_STUB_AVR_IO_H
#define HOST_STUB_AVR_IO_H

#include <stdint.h>

/* MSVC understands neither GNU attributes nor GNU inline asm.  The one
 * place this firmware uses either is sys_stack_paint(), which the host
 * build compiles out with STACK_GUARD=0. */
#ifdef _MSC_VER
#define __attribute__(x)
#endif

/* ATmega32: internal SRAM 0x0060..0x085F */
#define RAMEND 0x085F

/* Fake I/O space.  Declared here, defined once in the test. */
extern uint8_t H_SREG, H_MCUCSR;
extern uint8_t H_TCCR0, H_OCR0, H_TIMSK, H_TCCR2, H_OCR2, H_TCNT2;
extern uint8_t H_PORTB, H_DDRB, H_PINB;
extern uint8_t H_PORTC, H_DDRC, H_PINC;
extern uint8_t H_PORTD, H_DDRD, H_PIND;

#define SREG    H_SREG
#define MCUCSR  H_MCUCSR
#define TCCR0   H_TCCR0
#define OCR0    H_OCR0
#define TIMSK   H_TIMSK
#define TCCR2   H_TCCR2
#define OCR2    H_OCR2
#define TCNT2   H_TCNT2
#define PORTB   H_PORTB
#define DDRB    H_DDRB
#define PINB    H_PINB
#define PORTC   H_PORTC
#define DDRC    H_DDRC
#define PINC    H_PINC
#define PORTD   H_PORTD
#define DDRD    H_DDRD
#define PIND    H_PIND

/* bit positions */
#define PB0 0
#define PB1 1
#define PB2 2
#define PB3 3
#define PB4 4
#define PB5 5
#define PB6 6
#define PB7 7
#define PC0 0
#define PC1 1
#define PD1 1
#define PD2 2
#define PD4 4
#define PD5 5
#define PD7 7

/* MCUCSR reset-cause flags, ATmega32 positions */
#define PORF  0
#define EXTRF 1
#define BORF  2
#define WDRF  3

/* Timer0 CTC bits */
#define WGM01 3
#define CS00  0
#define CS01  1
#define OCIE0 1
#define WGM21 3
#define CS22  2
#define OCIE2 7
#define TOIE2 6

#endif
