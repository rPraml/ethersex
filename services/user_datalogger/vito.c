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
#include "datalogger.h"


#define KW_START 0x01
#define KW_END 0x04
#define KW_SYNC 0x05



volatile unsigned int vito_timeout_counter;
void vito_timeout(void) {
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
	//usart_tx_buffer()[0] = 0x04; // sync
	//usart_tx_start(5, NULL);
	vito_timeout_counter = 20;
  while (vito_timeout_counter) {
	 if (viessmann_readChar() == KW_SYNC) {
     VITO_DEBUG("Synced!\n");
     return 1;
   }
  }
  return 0;
}

static volatile char vito_txc_flag;
void vito_txc(void) {
	vito_txc_flag = 1;
}

// https://github.com/steand/optolink/blob/f45524259ba109d9a42e805e0c3667d77938a8af/src/main/java/de/myandres/optolink/ViessmannKW.java

int16_t vito_getData(int mode, uint16_t addr, int len, uint32_t data, char *buf) {

	int16_t ch,i, datalen;
  VITO_DEBUG("Mode: %02x, Addr: %04x, Len %02x\n", mode, addr, len);
	if (!vito_sync()) return -1;
	char *ptr = usart_tx_buffer();
	*ptr++ = KW_START;
	*ptr++ = mode;
	
	switch (mode) {
		// Lesebefehle
	case 0xF7: // Virtuell_Read (normales Lesen)
  case 0x6B: // 6B=GFA_Read
  case 0x7B: // 7B=PROZESS_READ
	  *ptr++ = addr >> 8;
    *ptr++ = addr;
    *ptr++ = len;
    datalen = 6;
    break;
  
  case 0xC7: // VIRTUAL READ
	case 0xCB: // PHYSICAL READ
	case 0xAE: // EEPROM READ
	case 0xC5: // PHYSICAL XRAM READ
	case 0x6E: // PHYSICAL PORT READ
	case 0x9E: // PHYSICAL BE READ
	case 0x33: // PHYSICAL KMBUS RAM READ
	case 0x43: // PHYSICAL KMBUS EEPROM READ
    *ptr++ = addr;
	  *ptr++ = len;
	  datalen = 5;
		break;
    
    // writes not yet implemented!
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

	*ptr++ = KW_END;
	vito_txc_flag = 0;
	usart_tx_start(datalen, vito_txc) ;
	while(!vito_txc_flag) { wdt_kick(); _delay_ms(1); }
	
	vito_timeout_counter = 4;
	for (i = 0; i < len; i++) {
		ch = viessmann_readChar();
		if (ch != 0x05) vito_timeout_counter = 4; // do not retrigger on periodic sync!
		if (ch < 0) return -1;
		snprintf_P(&buf[i*5], 6, PSTR("0x%02x,"),ch);
	}
  VITO_DEBUG("RX; %s\n", buf);
	return len * 5 - 1;
}

int16_t datalogger_vito_ecmd(char *cmd, char *output, uint16_t len) 
{
  int ret, fnct, addr, dataLen;
	uint32_t data;
  if (datalogger_state != DATALOGGER_STATE_IDLE) {
    output[0] = ECMD_NO_NEWLINE;
    return ECMD_AGAIN(0);
  }
	sscanf_P(cmd, PSTR("%x %x %x %lx"), &fnct, &addr, &dataLen, &data);
	if (dataLen * 5 > len) return ECMD_FINAL(snprintf_P(output, len, PSTR("Too long")));

  
	timeout_counter = 10;
	datalogger_setmode(DATALOGGER_STATE_TX_VITO);
	usart_setbaud(BAUDRATE(4800),8,'E',2);
	usart_rx_start();
	if ((ret = vito_getData(fnct, addr,dataLen,data, output)) >0) {
		datalogger_setmode(DATALOGGER_STATE_IDLE);
		return ECMD_FINAL(ret);
	}
	datalogger_setmode(DATALOGGER_STATE_IDLE);
	return ECMD_FINAL(snprintf_P(output, len, PSTR("Error")));
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


/*
  -- Ethersex META --
  header(services/user_datalogger/vito.h))
  timer(50, vito_timeout())
*/