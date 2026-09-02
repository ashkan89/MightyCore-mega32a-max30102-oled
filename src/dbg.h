/* dbg.h -- UART0 diagnostic stream (TX only, PD1)
 *
 * The board is already wired to a host serial port for the urboot
 * bootloader, so anything that can flash it can read this with no extra
 * wiring.  The OLED can show a handful of numbers; it cannot show a time
 * series, and it certainly cannot show one while a finger is covering the
 * sensor.  Every fault chased in this firmware so far was found from these
 * lines and would have been guesswork without them, so it stays in the
 * build.  Set DBG_UART to 0 in config.h to compile it out entirely.
 *
 * TX only and poll-driven, so it cannot perturb the sample timing the beat
 * detector depends on beyond the bytes it spends: one line per second at
 * 38400 baud is about 40 ms of wall clock, from the main loop.
 */
#ifndef DBG_H
#define DBG_H
#include "config.h"

/* Why a crossing was accepted or thrown away, reported per beat. */
enum { BEAT_OK = 0, BEAT_SHORT, BEAT_LONG, BEAT_ACQ, BEAT_CONT, BEAT_AMP };

/* What the start-up channel probe concluded.  REVERSED is acted on, not just
 * reported: the driver's word order is corrected for the rest of the run. */
enum { PROBE_OK = 0, PROBE_REVERSED, PROBE_NORED, PROBE_NOIR, PROBE_BOTH };

#if DBG_UART

void dbg_init(void);
void dbg_beat(uint16_t ibi_ms, uint16_t amp, uint8_t code);
void dbg_loop(void);         /* once per main-loop pass, for the rate figure */
void dbg_service(void);      /* emits one line per second, from the main loop */
uint8_t dbg_channel_probe(void);/* one-shot: which FIFO word follows which LED */

#else
static inline void dbg_init(void)    { }
static inline void dbg_beat(uint16_t i, uint16_t a, uint8_t c)
{ (void)i; (void)a; (void)c; }
static inline void dbg_loop(void)    { }
static inline void dbg_service(void) { }
static inline uint8_t dbg_channel_probe(void) { return PROBE_OK; }
#endif

#endif
