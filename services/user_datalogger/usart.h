#ifndef DATALOGGER_USART_H
#define DATALOGGER_USART_H

#define BAUDRATE(a) (F_CPU/(16l*(long)a)  - 1)

void usart_setbaud(uint16_t baud, uint8_t charSize, char parity, uint8_t stopbits)  ;

void usart_rx_clear(void);
void usart_rx_start() ;
int16_t usart_rx_get();


char * usart_tx_buffer();
void usart_tx_start(int16_t len,  void *txc_callback);

#endif