#ifndef S0LOGGER_H
#define S0LOGGER_H
void s0_ovfTick();
void s0_start();
int s0_ready();
void s0_capture();
int16_t s0_process_rx(uint8_t ch);
void s0_process_rx_err(void);
int16_t s0_ecmd_status(char *cmd, char *output, uint16_t len);

void s0_init(void);
void s0logger_mainloop(void);
void s0logger_counter(void);
// Enable/disable Input capturer
#define S0_OFF() PIN_CLEAR(DATALOGGER_S0_MODE); TIMSK1 |= _BV(ICIE1)
#define S0_ON()  PIN_SET(DATALOGGER_S0_MODE);   TIMSK1 &= ~_BV(ICIE1)


#ifdef DEBUG_S0
#include "core/debug.h"
#define S0_DEBUG(str...) debug_printf ("S0: " str)
#else
#define S0_DEBUG(...)    ((void) 0)
#endif

#endif