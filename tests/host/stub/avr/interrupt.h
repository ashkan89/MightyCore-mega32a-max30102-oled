/* Host stand-in: interrupts do not exist, and the one ISR in the code
 * under test (the 1 ms tick) is driven directly by the test instead. */
#ifndef HOST_STUB_AVR_INTERRUPT_H
#define HOST_STUB_AVR_INTERRUPT_H
#define ISR(vec)  void vec(void)
#define TIMER0_COMP_vect host_tick_isr
#define TIMER2_COMP_vect host_t2_isr
#define INT0_vect        host_int0_isr
static void sei(void) { }
static void cli(void) { }
#endif
