#ifndef KACO_H
#define KACO_H
/**
 * Polls the ID on RS485 bus
 */
void kaco_start(int id);

/**
 * Callback for serial data
 */
int16_t kaco_process_rx(uint8_t ch);

/**
 * Callback for error recovery
 */
void kaco_process_rx_err(void);

/**
 * Polls the ID on RS485 bus
 */
void goodwe_start();

/**
 * Callback for serial data
 */
int16_t goodwe_process_rx(uint8_t ch);

/**
 * Callback for error recovery
 */
void goodwe_process_rx_err(void);

/**
 * Init function
 */
void kaco_init(void);

/**
 * Returns the total AC power of all inverters
 */
int32_t kaco_get_total_power();
int32_t goodwe_get_total_power();

/**
 * Ecmd function
 */
int16_t kaco_ecmd_status(char *cmd, char *output, uint16_t len);

void kaco_mainloop(void);

#define RS485_0_OFF() PIN_CLEAR(DATALOGGER_RS485_0_TX_ENABLE); PIN_SET(DATALOGGER_RS485_0_RX_DISABLE)
#define RS485_0_TX()  PIN_SET(DATALOGGER_RS485_0_TX_ENABLE);   PIN_SET(DATALOGGER_RS485_0_RX_DISABLE)
#define RS485_0_RX()  PIN_CLEAR(DATALOGGER_RS485_0_TX_ENABLE); PIN_CLEAR(DATALOGGER_RS485_0_RX_DISABLE)

#define RS485_1_OFF() PIN_CLEAR(DATALOGGER_RS485_1_TX_ENABLE); PIN_SET(DATALOGGER_RS485_1_RX_DISABLE)
#define RS485_1_TX()  PIN_SET(DATALOGGER_RS485_1_TX_ENABLE);   PIN_SET(DATALOGGER_RS485_1_RX_DISABLE)
#define RS485_1_RX()  PIN_CLEAR(DATALOGGER_RS485_1_TX_ENABLE); PIN_CLEAR(DATALOGGER_RS485_1_RX_DISABLE)

#define DATALOGGER_STATE_TX_KACO 3
#define DATALOGGER_STATE_TX_GOODWE 4

#ifdef DEBUG_KACO
#include "core/debug.h"
#define KACO_DEBUG(str...) debug_printf ("KACO: " str)
#else
#define KACO_DEBUG(...)    ((void) 0)
#endif

#endif