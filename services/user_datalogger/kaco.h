#ifndef KACO_H
#define KACO_H
void kaco_start(int id);
int16_t kaco_process_rx(uint8_t ch);
void kaco_process_rx_err(void);
int16_t kaco_init(void);
int32_t kaco_get_total_power();
#endif