#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include "config.h"
#include "protocols/ecmd/ecmd-base.h"
#include "vito.h"
#define USE_USART DATA_LOGGER_USE_USART
#include "usart.h"



volatile unsigned int vito_timeout_counter;
void vito_ovfTick(void) {
    if (vito_timeout_counter)  vito_timeout_counter--;
}

// -------------- Viessmann
int16_t viessmann_readChar(void) {
	int16_t ch;
	while (vito_timeout_counter) {
		wdt_kick();
		_delay_ms(1);
		ch = usart_rx_get();
		if (ch > 0) return ch;
	}
	return -1;
}


int16_t vito_sync(void) {
	usart_tx_buffer()[0] = 0x04; // sync
	usart_tx_start(5, NULL);
	vito_timeout_counter = 10;
	return viessmann_readChar() == 0x05;
}

static volatile char vito_txc_flag;
void vito_txc(void) {
	vito_txc_flag = 1;
}
int16_t vito_getData(int mode, uint16_t addr, int len, uint32_t data, char *buf) {

	int16_t ch,i, datalen;
	if (!vito_sync()) return -1;
	char *ptr = usart_tx_buffer();
	*ptr++ = 0x01;
	*ptr++ = mode;
	*ptr++ = addr;
	*ptr++ = len;
	datalen = 5;
	
	switch (mode) {
		// Lesebefehle
	case 0xC7:
	case 0xCB:
	case 0xAE:
	case 0xC5:
	case 0x6E:
	case 0x9E:
	case 0x33:
	case 0x43:
		// read commands
		break;
	/*case 0xC8:

		for (datalen = 0; datalen < len; datalen++) {
			*ptr++ = data & 0xFF;
			data = data >> 8;
		}
		datalen =5 + len;
		break;*/
	default:
		return -1;
	}

	*ptr++ = 0x04;
	vito_txc_flag = 0;
	usart_tx_start(datalen, vito_txc) ;
	while(!vito_txc_flag) { wdt_kick(); _delay_ms(1); }
	
	vito_timeout_counter = 4;
	for (i = 0; i < len; i++) {
		ch = viessmann_readChar();
		if (ch != 0x05) vito_timeout_counter = 4;
		if (ch < 0) return -1;
		snprintf_P(&buf[i*5], 6, PSTR("0x%02x,"),ch);
	}
	return len*5-1;
}


/*
  This function will be called on request by menuconfig, if wanted...
  You need to enable ECMD_SUPPORT for this.
  Otherwise you can use this function for anything you like
int16_t
datalogger_vito_ecmd(char *cmd, char *output, uint16_t len) {

	int ret, fnct, addr, dataLen;
	uint32_t data;
	sscanf_P(cmd, PSTR("%x %x %x %lx"), &fnct, &addr, &dataLen, &data);
	if (dataLen * 5 > len) return ECMD_FINAL(snprintf_P(output, len, PSTR("Too long")));


	datalogger_viessmann_start();
	if ((ret = viessmann_getData(fnct, addr,dataLen,data, output)) >0) {
		datalogger_setmode(DATALOGGER_STATE_IDLE);
		return ECMD_FINAL(ret);
	}
	datalogger_setmode(DATALOGGER_STATE_IDLE);
	//strncpy(output, datalogger_ident, len);
	return ECMD_FINAL(snprintf_P(output, len, PSTR("Timeout")));
}
*/