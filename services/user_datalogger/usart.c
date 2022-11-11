#include <avr/pgmspace.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <util/parity.h>
#include "config.h"
#include "usart.h"

#define USE_USART DATA_LOGGER_USE_USART

#include "core/usart.h"
//generate_usart_init()


#define USART_RX_BUFFER_LEN 16
#define USART_RX_BUFFER_MASK 0x0F
#define USART_TX_BUFFER_LEN 64

static uint8_t rx_buffer[USART_RX_BUFFER_LEN];
static volatile uint8_t rx_write_ptr, rx_read_ptr;
static uint8_t tx_buffer[USART_TX_BUFFER_LEN + 2];
static volatile uint8_t tx_len, tx_sent;

void (* usart_on_tx_complete)();

// the receiver code
void usart_rx_clear(void) {
	rx_write_ptr = rx_read_ptr = 0;
}

// dies ist eine Art Ringpuffer
int16_t
usart_rx_put(uint8_t ch) {
	//if (((recv_write_ptr + 1) & USART_RECV_BUFFER_MASK) == recv_read_ptr) return -1; // buffer full
	rx_buffer[rx_write_ptr] = ch;
	rx_write_ptr = (rx_write_ptr + 1) & USART_RX_BUFFER_MASK;
	return 0;
}

int16_t
usart_rx_get() {
	if (rx_read_ptr == rx_write_ptr) return -1; // buffer empty
	uint8_t ret = rx_buffer[rx_read_ptr];
	rx_read_ptr = (rx_read_ptr + 1) & USART_RX_BUFFER_MASK;
	return ret;
}

// ISR-Routinen
ISR(usart(USART,_RX_vect)) {
	/* Ignore errors */
	uint8_t flags = usart(UCSR,A);
	if ((flags & _BV(usart(DOR))) || (usart(UCSR,A) & _BV(usart(FE)))) {
		uint8_t v = usart(UDR);
		(void) v;
		return;
	}
	uint8_t data = usart(UDR);
	usart_rx_put(data);
}

ISR(usart(USART,_TX_vect)) {
	if (tx_len && tx_sent < tx_len) { // bytes in buffer that aren't sent
		while (!(usart(UCSR,A) & _BV(usart(UDRE))));
		usart(UDR) = tx_buffer[tx_sent++];
	} else {
		/* Disable this interrupt, execute callback */
		tx_len = 0;
		usart(UCSR, B) |= _BV(usart(RXCIE)) | _BV(usart(RXEN)); // fullduplex not yet supported (and also not needed)
		
		if (usart_on_tx_complete)(*usart_on_tx_complete)();
		usart_on_tx_complete = 0;
	}
}
void usart_rx_start() {
	uint8_t sreg = SREG;
	
	usart_rx_clear();
	
	cli();	
	usart(UCSR, B) |= _BV(usart(RXCIE)) | _BV(usart(RXEN));
	SREG = sreg;
}

uint8_t * usart_tx_buffer() {
	return tx_buffer;
}
void usart_tx_start(int16_t len, void *txc_callback) {
	uint8_t sreg = SREG;
	// todo check if tx_len = 0;
	if (len <= 0) return;
	
	cli();
	/* enable transmitter only, bidi not yet supported (and not needed) */
	usart(UCSR, B) = _BV(usart(TXCIE)) | _BV(usart(TXEN));
	/* reset tx interrupt flag */
	usart(UCSR, A) |= _BV(usart(TXC));
	usart_on_tx_complete = txc_callback;
	/* Go! */
	SREG = sreg;

	tx_len = len;
	tx_sent = 1;
	usart(UDR) = tx_buffer[0];	// send char
}

void usart_setbaud(uint16_t baud, uint8_t charSize, char parity, uint8_t stopbits) {
	uint8_t sreg = SREG;
	cli();
	uint8_t out;
	switch(parity) {
		case 'E': out |= (1 << usart(UPM,1)); break;
		case 'O': out |= (1 << usart(UPM,1)) | (1 << usart(UPM,0)) ;
	}
	switch (charSize) {
		case 7: out |= (1 << usart(UCSZ,1)) ;	break;
		case 8: out |= (1 << usart(UCSZ,1)) | (1 << usart(UCSZ,0));	break;
	}
	if (stopbits == 2) out |= (1 << usart(USBS));
	usart(UCSR, C) = _BV_URSEL | out;
	usart(UBRR,H) = (baud >> 8);
	usart(UBRR,L) = (baud & 0xFF); 
	SREG = sreg;
}

