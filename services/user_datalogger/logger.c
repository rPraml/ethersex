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


#ifdef MQTT_SUPPORT
#include "protocols/mqtt/mqtt.h"
#endif

#ifdef DATA_LOGGER_S0
#include "s0logger.h"
#endif

#ifdef DATA_LOGGER_VITO
#include "vito.h"
#endif

#ifdef DATA_LOGGER_KACO
#include "kaco.h"
#endif




static volatile uint32_t gas_counter = 0;
static volatile uint32_t gas_counter_mqtt = 0;
static volatile int32_t gas_power = -1;
// Handle Pinchange Interrupt on PortD
/*ISR(PCINT3_vect)
{
  uint8_t TempDiff;
  uint8_t PinState = PINA;
  // Detect changes having proved stable:
  TempDiff = 
    // Bits that have changed (filter unchanged Bits)
    (PinState ^ stateD)
  
  if (!(PIND & _BV(PD7))) {
    gas_counter++;
  }
  
  stateD = PinState
}

void datalogger_init(void) {
  DDRD &= ~_BV(PD7);
  PORTD |= _BV(PD7);
  PCMSK3 = _BV(PD7);  // Enable Pinchange Interrupt on PortD
  PCICR |= 1<<PCIE3;
}
*/

void datalogger_init(void) {
 // DDRD &= ~_BV(PD7);
 // DDRD |= _BV(PD7);
 // PORTD &= ~_BV(PD7);
}

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
			RS485_0_OFF();
			RS485_1_OFF();
#endif			
			PIN_SET(STATUSLED_DATALOGGER_ACT);
			break;

#ifdef DATA_LOGGER_S0    
		case DATALOGGER_STATE_TX_S0:
      LOGGER_DEBUG("TX_S0\n");
#ifdef DATA_LOGGER_VITO
			VITO_OFF();
#endif			
#ifdef DATA_LOGGER_KACO
			RS485_0_OFF();
			RS485_1_OFF();
#endif			
			S0_ON();
			PIN_CLEAR(STATUSLED_DATALOGGER_ACT);
			break;
#endif

#ifdef DATA_LOGGER_VITO
		case DATALOGGER_STATE_TX_VITO:
      LOGGER_DEBUG("TX_VITO\n");
#ifdef DATA_LOGGER_KACO
			RS485_0_OFF();
			RS485_1_OFF();
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
			RS485_0_TX();
			RS485_1_OFF();
			PIN_CLEAR(STATUSLED_DATALOGGER_ACT);
			break;
		case DATALOGGER_STATE_TX_GOODWE:
      LOGGER_DEBUG("TX_GOODWE\n");
#ifdef DATA_LOGGER_VITO
			VITO_OFF();
#endif			
#ifdef DATA_LOGGER_S0 
			S0_OFF();
#endif
			RS485_0_OFF();
			RS485_1_TX();
			PIN_CLEAR(STATUSLED_DATALOGGER_ACT);
			break;
#endif
		}
	}
}

uint32_t datalogger_get_gas_power() {
  uint32_t ret = -1;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if (gas_power != -1) {
      ret = gas_power;
      gas_power = -1;
    }
  }
  return ret;
}
int16_t datalogger_mainloop(void) {
    // die Mainloop, wird sooft wie m�glich aufgerufenx

  
#ifdef MQTT_SUPPORT
  uint32_t power = datalogger_get_gas_power();
  if (power != -1) {
    char buf[16];
    uint8_t buf_length;
    LOGGER_DEBUG("Power:%ld\n", power);
    power = ((uint32_t)3600 * 100 * CONF_MTICKS_PER_SEC) / power;
    LOGGER_DEBUG("Power:%ld\n", power);
    buf_length = snprintf_P(buf, 16, PSTR("%ld"), power);
    mqtt_construct_publish_packet_P(PSTR("tele/gas-power/value"), buf, buf_length, false);
  }
#endif


	if (datalogger_state != DATALOGGER_STATE_IDLE && timeout_counter == 0) {
		// we ran in a timeout
		if (datalogger_rx_errback) {
			(*datalogger_rx_errback)();
		}
		LOGGER_DEBUG("Timeout. State %02x\n", datalogger_state);
		// PIN_CLEAR(STATUSLED_DEBUG); // RED Led on.
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
				// PIN_CLEAR(STATUSLED_DEBUG); // RED Led on.
			}
		} else if (ch > 0) {
			timeout_counter = ch;
		}
	}

	return ECMD_FINAL_OK;
}

void logger_mqtt_poll_cb() {
  uint32_t value;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if (gas_counter_mqtt == 0) {
      return;
    }
    value = gas_counter_mqtt;
    gas_counter_mqtt = 0;
  }
  char buf[16];
  uint8_t buf_length;
  buf_length = snprintf_P(buf, 16, PSTR("%ld"), value);
  mqtt_construct_publish_packet_P(PSTR("tele/gas/value"), buf, buf_length, false);
}

void datalogger_debug(void) {
  LOGGER_DEBUG("Gas: %ld %x %x %x %x\n", gas_counter, PINA, PINB, PINC, PIND);
}
// called every second.
void
datalogger_periodic(void) {
	static int id;
	PIN_TOGGLE(STATUSLED_DEBUG);
  if (datalogger_state == DATALOGGER_STATE_IDLE) {
    if (id == 0) {
      gas_counter_mqtt = gas_counter;
#ifdef MQTT_SUPPORT

#endif
			// Hier z�hlen wir die IDs durch. ID=0 ist der S0 - restl. IDs Kaco
      
#ifdef DATA_LOGGER_S0
			if (s0_ready()) {
				timeout_counter = PERIODIC_MS2MTICKS(5000); // 5 sec
				datalogger_setmode(DATALOGGER_STATE_TX_S0);
				datalogger_rx_callback = s0_process_rx;
				datalogger_rx_errback = s0_process_rx_err;
				s0_start();
	    }
#endif
    } else if (id <= DATA_LOGGER_KACO_MAX) {
#ifdef DATA_LOGGER_KACO
			timeout_counter = PERIODIC_MS2MTICKS(1000); // 1 sec
			datalogger_setmode(DATALOGGER_STATE_TX_KACO);
			datalogger_rx_callback = kaco_process_rx;
			datalogger_rx_errback  = kaco_process_rx_err;
			kaco_start(id);
#endif
    } else if (id == DATA_LOGGER_KACO_MAX + 1) {
	    // Nach KACO 1x den Goodwe abfragen
			timeout_counter = PERIODIC_MS2MTICKS(1000); // 1 sec
			datalogger_setmode(DATALOGGER_STATE_TX_GOODWE);
			datalogger_rx_callback = goodwe_process_rx;
			datalogger_rx_errback  = goodwe_process_rx_err;
			goodwe_start();
    } else {
      // als letztes dann noch die Vito-Kommunikation
      timeout_counter = PERIODIC_MS2MTICKS(1500); // sync should happen in one second
      datalogger_setmode(DATALOGGER_STATE_TX_VITO);
      vito_start();
      datalogger_rx_callback = vito_process_rx;
      datalogger_rx_errback = vito_process_err;
    }
    id++;
    if (id > DATA_LOGGER_KACO_MAX + 2) id = 0;
	}
}

/**
 * millitick hander
 */
void
datalogger_timeout(void) {
  uint8_t i;
  static uint32_t gas_timer;
  static uint8_t stateD;
  static uint8_t debounceD1;
  static uint8_t debounceD2;
	
  if (timeout_counter) {
		timeout_counter--;	
	}
  gas_timer++;
  
  i = stateD ^ ~PIND;
  debounceD1 = ~(debounceD1 & i);
  debounceD2 = debounceD1 ^ (debounceD2 & i);
  i &= debounceD1 & debounceD2;
  stateD ^= i;
  i = stateD & i;

  if (i & _BV(PD7)) {
    gas_counter++;
    // 100 Pulses pro m^3
    // 1m^3 ~ 10KWh
    // 1 pulse ~ 100wh
    // 1 pulse / h = 100w 
    gas_power = gas_timer;
    gas_timer = 0;
  }
}

const mqtt_callback_config_t logger_mqtt_callback_config PROGMEM = {
  .topic = NULL,
  .connack_callback = NULL,
  .poll_callback = logger_mqtt_poll_cb,
  .close_callback = NULL,
  .publish_callback = NULL
};
//   timer(20, datalogger_debug())
/*
  -- Ethersex META --
  header(services/user_datalogger/datalogger.h)
  init(datalogger_init)
  mainloop(datalogger_mainloop)
  timer(200, datalogger_periodic())
  periodic_milliticks_header(services/user_datalogger/datalogger.h)
  periodic_milliticks_isr(datalogger_timeout())
  mqtt_conf(logger_mqtt_callback_config)
*/
