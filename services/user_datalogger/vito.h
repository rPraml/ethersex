#ifndef VITO_H
#define VITO_H

void vito_init(void);
int16_t vito_getData(int mode, uint16_t addr, int len, uint32_t data, char *buf);

int16_t datalogger_vito_ecmd(char *cmd, char *output, uint16_t len);

void vito_start(void);
int16_t vito_process_rx(uint8_t ch);
void vito_process_err(void);

#define VITO_OFF() PIN_CLEAR(DATALOGGER_VIESS_MODE)
#define VITO_ON()  PIN_SET(DATALOGGER_VIESS_MODE)

#define DATALOGGER_STATE_TX_VITO 2

#ifdef DEBUG_VITO
#include "core/debug.h"
#define VITO_DEBUG(str...) debug_printf ("VITO: " str)
#else
#define VITO_DEBUG(...)    ((void) 0)
#endif

#endif