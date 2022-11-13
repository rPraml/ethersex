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
 * Init function
 */
void kaco_init(void);

/**
 * Returns the total AC power of all inverters
 */
int32_t kaco_get_total_power();

/**
 * Ecmd function
 */
int16_t kaco_ecmd_status(char *cmd, char *output, uint16_t len);

#define RS485_OFF() PIN_CLEAR(DATALOGGER_RS485_TX_ENABLE); PIN_SET(DATALOGGER_RS485_RX_DISABLE)
#define RS485_TX()  PIN_SET(DATALOGGER_RS485_TX_ENABLE);   PIN_SET(DATALOGGER_RS485_RX_DISABLE)
#define RS485_RX()  PIN_CLEAR(DATALOGGER_RS485_TX_ENABLE); PIN_CLEAR(DATALOGGER_RS485_RX_DISABLE)

#endif