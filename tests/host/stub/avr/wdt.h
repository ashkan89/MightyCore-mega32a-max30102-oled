#ifndef HOST_STUB_AVR_WDT_H
#define HOST_STUB_AVR_WDT_H
#define WDTO_2S 7
static void wdt_reset(void) { }
static void wdt_enable(int t) { (void)t; }
static void wdt_disable(void) { }
#endif
