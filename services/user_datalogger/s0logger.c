#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <util/parity.h>
#include "config.h"
#include "core/periodic.h"
#include "protocols/ecmd/ecmd-base.h"

#ifdef MQTT_SUPPORT
#include "protocols/mqtt/mqtt.h"
#endif

#include "util.h"
#include "usart.h"
#include "s0logger.h"
#ifdef DATA_LOGGER_KACO    
#include "kaco.h"
#endif
#define TICS_PER_HOUR 6328L
#define TICS_MAX 	  7000
#define TICS_MIN 	  300



#define CONVERT_FAIL 0
#define CONVERT_OK 1
/* Output des Zählers
/ACE0\3k260V01.18
"F.F(00)
C.1(1126110052708491)
C.5.0(00)
1.8.0(000052.5*kWh)
2.8.0(000172.3*kWh)
!
"
*/


static uint8_t datalogger_rx_state;

struct s0_status_t {
	uint16_t errorFlag;	// Bedeutung unbekannt
	uint16_t statusFlag;	// Bit 0 = Richtung, bit 1= Creep Flag
	uint8_t online; // 1 = Zähler online
	uint32_t energyFwd; // 100 Wh
	uint32_t energyRev; // 100 Wh
	int32_t watt;
  uint8_t pending :1;
} s0_status;


void s0_init() {
	TCCR1B |= _BV(ICNC1) | _BV(ICES1);   // NoiseCanceler + EdgeSelect
	TIMSK1 |= _BV(ICIE1);
}

#define START_CAPTURE 0
#define SYNCED 1
#define CAPTURING 2
#define CONVERSION_DONE 3
#define READ_SERIAL 4

volatile uint8_t pulse_state;
volatile uint8_t pulse_count;
volatile uint32_t pulse_timer;
volatile uint8_t pulse_capture_count;
volatile uint32_t pulse_capture_timer;

ISR(TIMER1_CAPT_vect)
{    //  Flanke an ICP pin

	switch (pulse_state) {
		
		case START_CAPTURE: // der erste Pulse wird ignoriert
    case SYNCED:
		pulse_count = 0;
		pulse_timer = 0;
		pulse_state++;
		break;
		
		case CAPTURING:
		pulse_count++;
		if (pulse_count > 3 && pulse_timer > 1000) {
			pulse_capture_count = pulse_count;
			pulse_capture_timer = pulse_timer;
			pulse_state = CONVERSION_DONE;
		}
		break;
	}

}
#ifdef MQTT_SUPPORT
bool
s0logger_mqtt_publish(void) {
  char buf[128];
  uint8_t buf_length;
  int32_t kaco_pwr = 0;
#ifdef DATA_LOGGER_KACO    
   kaco_pwr = kaco_get_total_power();
#endif
  
  buf_length = snprintf_P(buf, 128, 
    PSTR("{\"error\":%d,\"status\":%d,\"online\":%d,\"energyFwd\":%lu00,\"energyRev\":%lu00,\"power\":%ld,\"pvpower\":%ld}"),
    s0_status.errorFlag,
    s0_status.statusFlag, 
    s0_status.online,
    s0_status.energyFwd, 
    s0_status.energyRev, 
    s0_status.watt,
    kaco_pwr);
  return mqtt_construct_publish_packet_P(PSTR("tele/s0/value"), buf, buf_length, false);
 
}
#endif
void s0logger_mainloop(void) {
  #ifdef MQTT_SUPPORT
	if (mqtt_is_connected() && s0_status.pending) {
    if (s0logger_mqtt_publish()) {
      s0_status.pending = 0;
    } else {
      return; // queue is full
    }
  }
  #endif
}


void s0logger_counter() {
	pulse_timer++;
}
void s0_start() {
	util_readline('\n', READLINE_CRLF); // to reset buffer
	datalogger_rx_state = 0;
	
	memcpy_P(usart_tx_buffer(), PSTR("/?!\r\n"),5); // Initialisierung
	usart_setbaud(BAUDRATE(300),7,'E',1);
	usart_tx_start(5, NULL); // transmit 5 chars from buffer
	S0_DEBUG("Start\n");
}

int s0_ready() {
	S0_DEBUG("pulse_state %d\n", pulse_state);
	if (pulse_state == CONVERSION_DONE) {
		S0_DEBUG("P %d %d\n", pulse_capture_count, pulse_capture_timer);
		pulse_state = READ_SERIAL;
		return -1;
	}
	if (pulse_state == READ_SERIAL) {
		pulse_state = START_CAPTURE;
	}
    return 0; //pulse > 4;
}
/**
 * @return -1 wenn Fertig, 0 wenn nochmal, > 0 Dauer
 **/
int16_t s0_process_rx(uint8_t ch) {
	if (ch == 2) { // STX
		datalogger_rx_state = 10;
		return 0;
	}
	if (ch == 3) { // ETX
		s0_status.online = CONVERT_OK;
    s0_status.pending = 1;
		pulse_state = START_CAPTURE;
		return -1; // Fertig
	}
	uint16_t tmp_lo;
	uint32_t tmp_hi;

	char * line = util_readline(ch, READLINE_CRLF);
	if (!line) return 0; // no line yet there

	S0_DEBUG("State %d Line %s\n", datalogger_rx_state, line);
	
	switch (datalogger_rx_state) {
	case 0:
		_delay_ms(50);
		memcpy_P(usart_tx_buffer(), PSTR("\x06""000\r\n"),6); // copy init sequence into buffer
		usart_tx_start(6, NULL);
		datalogger_rx_state++;
		break;

	case 10:
		// hier werden die Zeilen vom Zähler geparsed
		
		sscanf_P(line, PSTR("FF(%x)"), 	&s0_status.errorFlag);
		sscanf_P(line, PSTR("C.5.0(%x)"), 	&s0_status.statusFlag);
		s0_status.watt = (uint32_t)(pulse_capture_count)*(uint32_t)(180000) / (uint32_t)pulse_capture_timer;
		if (s0_status.statusFlag & 0x01) s0_status.watt *=  -1;
		
		if (sscanf_P(line, PSTR("1.8.0(%lu.%1u*"), &tmp_hi, &tmp_lo) == 2) {
			s0_status.energyFwd = tmp_hi * 10 + tmp_lo;
		}
		if (sscanf_P(line, PSTR("2.8.0(%lu.%1u*"), &tmp_hi, &tmp_lo) == 2) {
			s0_status.energyRev  = tmp_hi * 10 + tmp_lo;
		}

		return PERIODIC_MS2MTICKS(2000); // timeout ~2 sec
	}
	return 0;
}

void s0_process_rx_err(void) {
	pulse_state = START_CAPTURE;
	s0_status.online = CONVERT_FAIL;
  s0_status.pending = 1;
}

int16_t
s0_ecmd_status(char *cmd, char *output, uint16_t len) {
	static uint8_t line;
    int32_t kaco_pwr;
	//uint16_t watt;
	if (cmd[0] != 23) {
		cmd[0] = 23;
		line = 0;
	}
	switch (line++) {
	case 0:
		/*
		1000 pulse = 1KWh
		1 Pulse = 1Wh
		Timerticks: 115200 / sec = F_CPU/64*3600 per hour	*/
		len = snprintf_P(output, len, PSTR("%4d,%4d,%3d,%10lu00,%10lu00,%8ld"),
		        s0_status.errorFlag, 
			s0_status.statusFlag, 
			s0_status.online,
      s0_status.energyFwd, 
      s0_status.energyRev, 
      s0_status.watt);
		
#ifdef DATA_LOGGER_KACO    
		output[len] = ECMD_NO_NEWLINE;
		return ECMD_AGAIN(len);
  case 1:
      kaco_pwr = kaco_get_total_power();
      len = snprintf_P(output, len, PSTR(",%10ld,%10ld"),
          kaco_pwr, kaco_pwr + s0_status.watt);

#endif
		return ECMD_FINAL(len);

	}
	return ECMD_FINAL_OK;
}

/*
  -- Ethersex META --
  header(services/user_datalogger/s0logger.h)
  mainloop(s0logger_mainloop)
  init(s0_init)
  periodic_milliticks_header(services/user_datalogger/s0logger.h)
  periodic_milliticks_isr(s0logger_counter())
*/

