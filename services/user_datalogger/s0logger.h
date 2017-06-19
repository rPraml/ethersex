#ifndef S0LOGGER_H
#define S0LOGGER_H
void s0_ovfTick();
void s0_start();
int s0_ready();
void s0_capture();
int16_t s0_process_rx(uint8_t ch);
void s0_process_rx_err(void);
int16_t s0_ecmd_status(char *cmd, char *output, uint16_t len);
#endif