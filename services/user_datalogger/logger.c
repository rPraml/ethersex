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
#include <util/atomic.h>
#include "config.h"
#include "datalogger.h"
#include "core/periodic.h"
#include "protocols/ecmd/ecmd-base.h"

#define USE_USART DATA_LOGGER_USE_USART
#include "usart.h"



#ifdef DATA_LOGGER_S0
#include "s0logger.h"
#endif

#ifdef DATA_LOGGER_VITO
#include "vito.h"
#endif

#ifdef DATA_LOGGER_KACO
#include "kaco.h"
#endif


int16_t (*datalogger_rx_callback)(uint8_t ch);

void (*datalogger_rx_errback)();


void datalogger_setmode(uint8_t state) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		datalogger_state = state;
		switch (datalogger_state) {
		case DATALOGGER_STATE_IDLE:
      LOGGER_DEBUG("IDLE\n");
			datalogger_rx_callback = NULL;
			datalogger_rx_errback = NULL;
#ifdef DATA_LOGGER_S0 
			S0_OFF();
#endif
#ifdef DATA_LOGGER_VITO
			VITO_OFF();
#endif			
#ifdef DATA_LOGGER_KACO
			RS485_OFF();
#endif			
			PIN_SET(STATUSLED_DATALOGGER_ACT);
			PIN_SET(STATUSLED_S0_ACT);
			break;

#ifdef DATA_LOGGER_S0    
		case DATALOGGER_STATE_TX_S0:
      LOGGER_DEBUG("TX_S0\n");
#ifdef DATA_LOGGER_VITO
			VITO_OFF();
#endif			
#ifdef DATA_LOGGER_KACO
			RS485_OFF();
#endif			
			S0_ON();
			PIN_CLEAR(STATUSLED_S0_ACT);
			PIN_CLEAR(STATUSLED_DATALOGGER_ACT);
			break;
#endif

#ifdef DATA_LOGGER_VITO
		case DATALOGGER_STATE_TX_VITO:
      LOGGER_DEBUG("TX_VITO\n");
#ifdef DATA_LOGGER_KACO
			RS485_OFF();
#endif	
#ifdef DATA_LOGGER_S0 
			S0_OFF();
#endif
			VITO_ON();
			PIN_CLEAR(STATUSLED_DATALOGGER_ACT);
			break;
#endif

#ifdef DATA_LOGGER_KACO
		case DATALOGGER_STATE_TX_KACO:
      LOGGER_DEBUG("TX_KACO\n");
#ifdef DATA_LOGGER_VITO
			VITO_OFF();
#endif			
#ifdef DATA_LOGGER_S0 
			S0_OFF();
#endif
			RS485_TX();
			PIN_CLEAR(STATUSLED_DATALOGGER_ACT);
			break;
#endif
		}
	}
}

int16_t datalogger_mainloop(void) {
    // die Mainloop, wird sooft wie möglich aufgerufenx
	
	if (datalogger_state != DATALOGGER_STATE_IDLE && timeout_counter == 0) {
		// we ran in a timeout
		LOGGER_DEBUG("Timeout\n", datalogger_state);
		if (datalogger_rx_errback) {
			(*datalogger_rx_errback)();
		}
		PIN_CLEAR(STATUSLED_DEBUG); // RED Led on.
		datalogger_setmode(DATALOGGER_STATE_IDLE);
	}
	
	int16_t ch = usart_rx_get();
	if (ch < 0) return ECMD_FINAL_OK; // negative, no character yet received

	if (datalogger_rx_callback) {
		ch = (*datalogger_rx_callback)(ch); // pass the character to the current RX-Callback. Callback returns -1 if there is a problem
		if (ch < 0) {
			LOGGER_DEBUG("State %02d, Res %02d\n", datalogger_state, ch);
			datalogger_setmode(DATALOGGER_STATE_IDLE);
			if (ch == FINISH_ERR) {
				
				PIN_CLEAR(STATUSLED_DEBUG); // RED Led on.
			}
		} else if (ch > 0) {
			timeout_counter = ch;
		}
	}

	return ECMD_FINAL_OK;
}

// called every second.
void
datalogger_periodic(void) {
	static int id;
	PIN_TOGGLE(STATUSLED_DATALOGGER_ACT);
  if (datalogger_state == DATALOGGER_STATE_IDLE) {
		PIN_SET(STATUSLED_DEBUG);
    if (id == 0) {
			// Hier zählen wir die IDs durch. ID=0 ist der S0 - restl. IDs Kaco
#ifdef DATA_LOGGER_S0
			if (s0_ready()) {
				timeout_counter = PERIODIC_MS2MTICKS(5000); // 5 sec
				datalogger_setmode(DATALOGGER_STATE_TX_S0);
				datalogger_rx_callback = s0_process_rx;
				datalogger_rx_errback = s0_process_rx_err;
				s0_start();
			}
#endif
    } else {
#ifdef DATA_LOGGER_KACO
			timeout_counter = PERIODIC_MS2MTICKS(1000); // 1 sec
			datalogger_setmode(DATALOGGER_STATE_TX_KACO);
			datalogger_rx_callback = kaco_process_rx;
			datalogger_rx_errback  = kaco_process_rx_err;
			kaco_start(id);
#endif
    }
    id++;
    if (id > DATA_LOGGER_KACO_MAX) id = 0;
	}
}

/**
 * millitick hander
 */
void
datalogger_timeout(void) {
	if (timeout_counter) {
		timeout_counter--;	
	}
}

/*
  -- Ethersex META --
  header(services/user_datalogger/datalogger.h)
  mainloop(datalogger_mainloop)
  timer(200, datalogger_periodic())
  periodic_milliticks_header(services/user_datalogger/datalogger.h)
  periodic_milliticks_isr(datalogger_timeout())
*/
