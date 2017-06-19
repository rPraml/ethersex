/*
 * Copyright (c) 2011 Roland Praml pram@gmx.de
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * For more information on the GPL, please go to:
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <avr/pgmspace.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "config.h"
#include "datalogger.h"

#include "protocols/ecmd/ecmd-base.h"

#define USE_USART DATA_LOGGER_USE_USART
#include "usart.h"


#define DATALOGGER_STATE_IDLE 0

#ifdef DATA_LOGGER_S0
#include "s0logger.h"
#define DATALOGGER_STATE_TX_S0 1
#endif

#ifdef DATA_LOGGER_VITO
#include "vito.h"
#define DATALOGGER_STATE_TX_VITO 2
#endif

#ifdef DATA_LOGGER_KACO
#include "kaco.h"
#define DATALOGGER_STATE_TX_KACO 3
#endif


volatile unsigned int timeout_counter;
volatile uint8_t datalogger_state, on_tx_complete_state;
void datalogger_setmode(uint8_t state);



int16_t (*datalogger_rx_callback)(uint8_t ch);
void (*datalogger_rx_errback)();


// Zeitmessungen für die S0-Pulsauswertung
static volatile uint8_t block_zeitmessung;
ISR(TIMER1_OVF_vect) { 	   // Timer1 Überlauf
	// timer = 1,7578125 Hz, t = 0,56888sec;
	if (timeout_counter)  timeout_counter--;
#ifdef DATA_LOGGER_S0
	s0_ovfTick();
#endif
#ifdef DATA_LOGGER_VITO
    vito_ovfTick();
#endif

}

#ifdef DATA_LOGGER_S0
ISR(TIMER1_CAPT_vect) {    //  Flanke an ICP pin
	s0_capture();
}
#endif


void datalogger_setmode(uint8_t state) {
	uint8_t sreg = SREG;
	cli();
	datalogger_state = state;
	switch (datalogger_state) {
	case DATALOGGER_STATE_IDLE:
		datalogger_rx_callback = NULL;
		datalogger_rx_errback = NULL;
		PIN_CLEAR(DATALOGGER_VIESS_MODE);
		PIN_CLEAR(DATALOGGER_RS485_TX_ENABLE);
		PIN_SET(DATALOGGER_RS485_RX_DISABLE);
		PIN_CLEAR(DATALOGGER_S0_MODE);
#ifdef DATA_LOGGER_S0    
		TIMSK |= (1<<TOIE1) | (1<<TICIE1);   // overflow und Input-capture aktivieren, Mega32: TIMSK
		TCCR1B |= (1<<ICNC1) + (1 << ICES1);
#else
		TIMSK |= (1<<TOIE1);   // overflow und Input-capture aktivieren, Mega32: TIMSK
#endif
		TCCR1A = 0;                      // normal mode, keine PWM Ausgänge
		PIN_SET(STATUSLED_DATALOGGER_ACT);
        PIN_SET(STATUSLED_S0_ACT);
		break;

#ifdef DATA_LOGGER_S0    
	case DATALOGGER_STATE_TX_S0:
		PIN_CLEAR(STATUSLED_S0_ACT);
		TIMSK &= ~(1<<TICIE1);
		PIN_CLEAR(DATALOGGER_VIESS_MODE);
		PIN_CLEAR(DATALOGGER_RS485_TX_ENABLE);
		PIN_SET(DATALOGGER_RS485_RX_DISABLE);
		PIN_SET(DATALOGGER_S0_MODE);
		break;
#endif

#ifdef DATA_LOGGER_VITO
	case DATALOGGER_STATE_TX_VITO:
		PIN_CLEAR(STATUSLED_DATALOGGER_ACT);
		TIMSK &= ~(1<<TICIE1);
		PIN_SET(DATALOGGER_VIESS_MODE);
		PIN_CLEAR(DATALOGGER_RS485_TX_ENABLE);
		PIN_SET(DATALOGGER_RS485_RX_DISABLE);
		PIN_CLEAR(DATALOGGER_S0_MODE);
		break;
#endif

#ifdef DATA_LOGGER_KACO
    case DATALOGGER_STATE_TX_KACO:
		PIN_CLEAR(STATUSLED_DATALOGGER_ACT);
		TIMSK &= ~(1<<TICIE1);
		PIN_CLEAR(DATALOGGER_VIESS_MODE);
		PIN_SET(DATALOGGER_RS485_TX_ENABLE);
		PIN_SET(DATALOGGER_RS485_RX_DISABLE);
		PIN_CLEAR(DATALOGGER_S0_MODE);
		break;
#endif
	}
	SREG = sreg;
}

int16_t datalogger_init(void) {
	DATALOGGERDEBUG("init\n");
	datalogger_setmode(DATALOGGER_STATE_IDLE);
	return ECMD_FINAL_OK;
}

int16_t datalogger_mainloop(void) {
    // die Mainloop, wird sooft wie möglich aufgerufen
	if (datalogger_state != DATALOGGER_STATE_IDLE) {
		if (timeout_counter == 0) {
			if (datalogger_rx_errback) (*datalogger_rx_errback)();
			datalogger_setmode(DATALOGGER_STATE_IDLE);
		}
	}
	int16_t ch = usart_rx_get();
	if (ch < 0) return ECMD_FINAL_OK; // negative, no character yet received

	if (datalogger_rx_callback) {
		ch = (*datalogger_rx_callback)(ch); // pass the character to the current RX-Callback. Callback returns -1 if there is a problem
		if (ch < 0) {
			datalogger_setmode(DATALOGGER_STATE_IDLE);
		} else if (ch > 0) {
			timeout_counter = ch;
		}
	}

	return ECMD_FINAL_OK;
}

#ifdef DATA_LOGGER_S0
void datalogger_s0_start(void) {
    if (!s0_ready()) return;
	timeout_counter = 5;
	datalogger_setmode(DATALOGGER_STATE_TX_S0);
	s0_start();
	datalogger_rx_callback = s0_process_rx;
	datalogger_rx_errback = s0_process_rx_err;
}
#endif

#ifdef DATA_LOGGER_VITO
void datalogger_vito_start(void) {
	while (datalogger_state != DATALOGGER_STATE_IDLE) {
		wdt_kick();
		datalogger_mainloop();
	}
    _delay_ms(10);
	timeout_counter = 10;
	datalogger_setmode(DATALOGGER_STATE_TX_VITO);
	usart_setbaud(BAUDRATE(4800),8,'E',2);
	usart_rx_start() ;
}

int16_t datalogger_vito_ecmd(char *cmd, char *output, uint16_t len) 
{
  int ret, fnct, addr, dataLen;
	uint32_t data;
	sscanf_P(cmd, PSTR("%x %x %x %lx"), &fnct, &addr, &dataLen, &data);
	if (dataLen * 5 > len) return ECMD_FINAL(snprintf_P(output, len, PSTR("Too long")));

	datalogger_vito_start();
	if ((ret = vito_getData(fnct, addr,dataLen,data, output)) >0) {
		datalogger_setmode(DATALOGGER_STATE_IDLE);
		return ECMD_FINAL(ret);
	}
	datalogger_setmode(DATALOGGER_STATE_IDLE);
	//strncpy(output, datalogger_ident, len);
	return ECMD_FINAL(snprintf_P(output, len, PSTR("Timeout")));
}
#endif


#ifdef DATA_LOGGER_KACO
void datalogger_kaco_start(int id) {
	timeout_counter = 3;
	datalogger_setmode(DATALOGGER_STATE_TX_KACO);
	kaco_start(id);
	datalogger_rx_callback = kaco_process_rx;
	datalogger_rx_errback  = kaco_process_rx_err;
}
#endif

/*
  If enabled in menuconfig, this function is periodically called
  change "timer(100,app_sample_periodic)" if needed
*/
int16_t
datalogger_periodic(void) {
	static int timer;
    if (datalogger_state == DATALOGGER_STATE_IDLE) {
        if (timer == 0) {
            datalogger_s0_start();
        } else {
            datalogger_kaco_start(timer);
        }
        timer++;
        if (timer > DATA_LOGGER_KACO_MAX) timer = 0;
	}
// timerinterval is about 0.5sec

	return ECMD_FINAL_OK;
}




/*
  -- Ethersex META --
  header(services/user_datalogger/datalogger.h)
  mainloop(datalogger_mainloop)
  timer(128,datalogger_periodic())
  init(datalogger_init)
*/
